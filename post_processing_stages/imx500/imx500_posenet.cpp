/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_posenet.cpp - IMX500 inference for PoseNet
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/geometry.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include "imx500_post_processing_stage.hpp"

using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;
namespace controls = libcamera::controls;

namespace
{

constexpr Size INPUT_TENSOR_SIZE = { 481, 353 };
constexpr Size MAP_SIZE = { 31, 23 };
constexpr unsigned int NUM_KEYPOINTS = 17;
// These 16 edges allow traversing of the pose graph along the mid_offsets (see
// paper for details).
static constexpr int NUM_EDGES = 16;
constexpr unsigned int STRIDE = 16;
constexpr unsigned int NUM_HEATMAPS = NUM_KEYPOINTS * MAP_SIZE.width * MAP_SIZE.height;
constexpr unsigned int NUM_SHORT_OFFSETS = 2 * NUM_KEYPOINTS * MAP_SIZE.width * MAP_SIZE.height;
constexpr unsigned int NUM_MID_OFFSETS = 64 * MAP_SIZE.width * MAP_SIZE.height;

enum KeypointType
{
	Nose,
	LeftEye,
	RightEye,
	LeftEar,
	RightEar,
	LeftShoulder,
	RightShoulder,
	LeftElbow,
	RightElbow,
	LeftWrist,
	RightWrist,
	LeftHip,
	RightHip,
	LeftKnee,
	RightKnee,
	LeftAnkle,
	RightAnkle
};

const std::array<std::pair<KeypointType, KeypointType>, 32> EdgeList = { {
	// Forward edges
	{ Nose, LeftEye },
	{ LeftEye, LeftEar },
	{ Nose, RightEye },
	{ RightEye, RightEar },
	{ Nose, LeftShoulder },
	{ LeftShoulder, LeftElbow },
	{ LeftElbow, LeftWrist },
	{ LeftShoulder, LeftHip },
	{ LeftHip, LeftKnee },
	{ LeftKnee, LeftAnkle },
	{ Nose, RightShoulder },
	{ RightShoulder, RightElbow },
	{ RightElbow, RightWrist },
	{ RightShoulder, RightHip },
	{ RightHip, RightKnee },
	{ RightKnee, RightAnkle },

	// Backward edges
	{ LeftEye, Nose },
	{ LeftEar, LeftEye },
	{ RightEye, Nose },
	{ RightEar, RightEye },
	{ LeftShoulder, Nose },
	{ LeftElbow, LeftShoulder },
	{ LeftWrist, LeftElbow },
	{ LeftHip, LeftShoulder },
	{ LeftKnee, LeftHip },
	{ LeftAnkle, LeftKnee },
	{ RightShoulder, Nose },
	{ RightElbow, RightShoulder },
	{ RightWrist, RightElbow },
	{ RightHip, RightShoulder },
	{ RightKnee, RightHip },
	{ RightAnkle, RightKnee },
} };

struct Point
{
	float y, x;
};

// An adjacency list representing the directed edges connecting keypoints.
struct AdjacencyList
{
	explicit AdjacencyList(const int num_nodes) : child_ids(num_nodes), edge_ids(num_nodes) {}

	// child_ids[i] is a vector holding the node ids of all children of the i-th
	// node and edge_ids[i] is a vector holding the edge ids of all edges stemming
	// from the i-th node. If the k-th edge in the graph starts at the i-th node
	// and ends at the j-th node, then child_ids[i] and edge_ids will contain j
	// and k, respectively, at corresponding positions.
	std::vector<std::vector<int>> child_ids;
	std::vector<std::vector<int>> edge_ids;
};

// Defines a 2-D keypoint with (x, y) float coordinates and its type id.
struct KeypointWithScore
{
	KeypointWithScore(const Point &_pt, const int _id, const float _score) : point(_pt), id(_id), score(_score) {}

	[[maybe_unused]] friend std::ostream &operator<<(std::ostream &ost, const KeypointWithScore &keypoint)
	{
		return ost << keypoint.point.y << ", " << keypoint.point.x << ", " << keypoint.id << ", " << keypoint.score;
	}

	bool operator<(const KeypointWithScore &other) const { return score < other.score; }
	bool operator>(const KeypointWithScore &other) const { return score > other.score; }

	Point point;
	int id;
	float score;
};

using KeypointQueue = std::priority_queue<KeypointWithScore, std::vector<KeypointWithScore>>;
using PoseKeypoints = std::array<Point, NUM_KEYPOINTS>;
using PoseKeypointScores = std::array<float, NUM_KEYPOINTS>;

struct PoseResults
{
	float pose_score;
	PoseKeypoints pose_keypoints;
	PoseKeypointScores pose_keypoint_scores;
};

float sigmoid(const float x)
{
	return 1.0f / (1.0f + std::exp(-x));
}

float log_odds(const float x)
{
	return -std::log(1.0f / (x + 1E-6) - 1.0f);
}

// Computes the squared distance between a pair of 2-D points.
float squared_distance(const Point &a, const Point &b)
{
	const float dy = b.y - a.y;
	const float dx = b.x - a.x;
	return dy * dy + dx * dx;
}

std::vector<float> format_tensor(const float *data, unsigned int size, unsigned int div)
{
	std::vector<float> tensor(size * MAP_SIZE.width * MAP_SIZE.height);

	for (unsigned int i = 0; i < size; i++)
	{
		for (unsigned int j = 0; j < MAP_SIZE.width; j++)
		{
			for (unsigned int k = 0; k < MAP_SIZE.height; k++)
			{
				tensor[(size * MAP_SIZE.width * k) + (size * j) + i] =
					data[(MAP_SIZE.width * MAP_SIZE.height * i) + (j * MAP_SIZE.height) + k] / div;
			}
		}
	}

	return tensor;
}

// Build an adjacency list of the pose graph.
AdjacencyList build_agency_list()
{
	AdjacencyList adjacency_list(NUM_KEYPOINTS);

	for (unsigned int k = 0; k < EdgeList.size(); ++k)
	{
		const int parent_id = EdgeList[k].first;
		const int child_id = EdgeList[k].second;
		adjacency_list.child_ids[parent_id].push_back(child_id);
		adjacency_list.edge_ids[parent_id].push_back(k);
	}

	return adjacency_list;
}

bool pass_keypoint_nms(std::vector<PoseKeypoints> poses, const size_t num_poses, const KeypointWithScore &keypoint,
					   const float squared_nms_radius)
{
	for (unsigned int i = 0; i < num_poses; ++i)
	{
		if (squared_distance(keypoint.point, poses[i][keypoint.id]) <= squared_nms_radius)
			return false;
	}

	return true;
}

// Finds the indices of the scores if we sort them in decreasing order.
void decreasing_arg_sort(std::vector<int> &indices, const float *scores, unsigned int num_scores)
{
	indices.resize(num_scores);
	std::iota(indices.begin(), indices.end(), 0);
	std::sort(indices.begin(), indices.end(), [&scores](const int i, const int j) { return scores[i] > scores[j]; });
}

// Finds the indices of the scores if we sort them in decreasing order.
void decreasing_arg_sort(std::vector<int> &indices, const std::vector<float> &scores)
{
	decreasing_arg_sort(indices, scores.data(), scores.size());
}

// Helper function for 1-D linear interpolation. It computes the floor and the
// ceiling of the input coordinate, as well as the weighting factor between the
// two interpolation endpoints, such that:
// y = (1 - x_lerp) * vec[x_floor] + x_lerp * vec[x_ceil]
void build_linear_interpolation(const float x, const int n, int *x_floor, int *x_ceil, float *x_lerp)
{
	const float x_proj = std::clamp(x, 0.0f, n - 1.0f);
	*x_floor = static_cast<int>(std::floor(x_proj));
	*x_ceil = static_cast<int>(std::ceil(x_proj));
	*x_lerp = x - (*x_floor);
}

// Helper function for 2-D bilinear interpolation. It computes the four corners
// of the 2x2 cell that contain the input coordinates (x, y), as well as the
// weighting factor between the four interpolation endpoints, such that:
// y =
//   (1 - y_lerp) * ((1 - x_lerp) * vec[top_left] + x_lerp * vec[top_right]) +
//   y_lerp * ((1 - x_lerp) * tensor[bottom_left] + x_lerp * vec[bottom_right])
void build_bilinear_interpolation(const Point point, const unsigned int num_channels, int *top_left, int *top_right,
								  int *bottom_left, int *bottom_right, float *y_lerp, float *x_lerp)
{
	int y_floor;
	int y_ceil;
	build_linear_interpolation(point.y, MAP_SIZE.height, &y_floor, &y_ceil, y_lerp);
	int x_floor;
	int x_ceil;
	build_linear_interpolation(point.x, MAP_SIZE.width, &x_floor, &x_ceil, x_lerp);
	*top_left = (y_floor * MAP_SIZE.width + x_floor) * num_channels;
	*top_right = (y_floor * MAP_SIZE.width + x_ceil) * num_channels;
	*bottom_left = (y_ceil * MAP_SIZE.width + x_floor) * num_channels;
	*bottom_right = (y_ceil * MAP_SIZE.width + x_ceil) * num_channels;
}

// Sample the input tensor values at position (x, y) and at multiple channels.
// The input tensor has shape [height, width, num_channels]. We bilinearly
// sample its value at tensor(y, x, c), for c in the channels specified. This
// is faster than calling the single channel interpolation function multiple
// times because the computation of the positions needs to be done only once.
std::vector<float> sample_tensor_at_multiple_channels(const std::vector<float> &tensor, const Point &point,
													  const std::vector<int> &result_channels,
													  unsigned int num_channels)
{
	int top_left, top_right, bottom_left, bottom_right;
	float y_lerp, x_lerp;

	build_bilinear_interpolation(point, num_channels, &top_left, &top_right, &bottom_left, &bottom_right, &y_lerp,
								 &x_lerp);

	std::vector<float> result;
	for (auto &c : result_channels)
	{
		result.push_back((1 - y_lerp) * ((1 - x_lerp) * tensor[top_left + c] + x_lerp * tensor[top_right + c]) +
						 y_lerp * ((1 - x_lerp) * tensor[bottom_left + c] + x_lerp * tensor[bottom_right + c]));
	}

	return result;
}

// Sample the input tensor values at position (x, y) and at a single channel.
// The input tensor has shape [height, width, num_channels]. We bilinearly
// sample its value at tensor(y, x, channel).
float sample_tensor_at_single_channel(const std::vector<float> &tensor, const Point &point, unsigned int num_channels,
									  const int c)
{
	std::vector<float> result = sample_tensor_at_multiple_channels(tensor, point, std::vector<int> { c }, num_channels);
	return result[0];
}

KeypointQueue build_keypoint_queue(const std::vector<float> &scores, const std::vector<float> &short_offsets,
								   const float score_threshold)
{
	constexpr unsigned int local_maximum_radius = 1;
	unsigned int score_index = 0;
	KeypointQueue queue;

	for (unsigned int y = 0; y < MAP_SIZE.height; ++y)
	{
		for (unsigned int x = 0; x < MAP_SIZE.width; ++x)
		{
			unsigned int offset_index = 2 * score_index;
			for (unsigned int j = 0; j < NUM_KEYPOINTS; ++j)
			{
				const float score = scores[score_index];
				if (score >= score_threshold)
				{
					// Only consider keypoints whose score is maximum in a local window.
					bool local_maximum = true;
					const unsigned int y_start = std::max((int)y - (int)local_maximum_radius, 0);
					const unsigned int y_end = std::min(y + local_maximum_radius + 1, MAP_SIZE.height);
					for (unsigned int y_current = y_start; y_current < y_end; ++y_current)
					{
						const unsigned int x_start = std::max((int)x - (int)local_maximum_radius, 0);
						const unsigned int x_end = std::min(x + local_maximum_radius + 1, MAP_SIZE.width);
						for (unsigned int x_current = x_start; x_current < x_end; ++x_current)
						{
							if (scores[y_current * MAP_SIZE.width * NUM_KEYPOINTS + x_current * NUM_KEYPOINTS + j] >
									score)
							{
								local_maximum = false;
								break;
							}
						}
						if (!local_maximum)
							break;
					}
					if (local_maximum)
					{
						const float dy = short_offsets[offset_index];
						const float dx = short_offsets[offset_index + NUM_KEYPOINTS];
						const float y_refined = std::clamp(y + dy, 0.0f, MAP_SIZE.height - 1.0f);
						const float x_refined = std::clamp(x + dx, 0.0f, MAP_SIZE.width - 1.0f);
						queue.emplace(Point { y_refined, x_refined }, j, score);
					}
				}

				++score_index;
				++offset_index;
			}
		}
	}

	return queue;
}

// Follows the mid-range offsets, and then refines the position by the short-
// range offsets for a fixed number of steps.
Point find_displaced_position(const std::vector<float> &short_offsets, const std::vector<float> &mid_offsets,
							  const Point &source, const int edge_id, const int target_id,
							  const int offset_refinement_steps)
{
	float y = source.y, x = source.x;

	// Follow the mid-range offsets.
	std::vector<int> channels = { edge_id, NUM_EDGES + edge_id };

	// Total size of mid_offsets is height x width x 2*2*num_edges
	std::vector<float> offsets = sample_tensor_at_multiple_channels(mid_offsets, source, channels, 2 * 2 * NUM_EDGES);
	y = std::clamp(y + offsets[0], 0.0f, MAP_SIZE.height - 1.0f);
	x = std::clamp(x + offsets[1], 0.0f, MAP_SIZE.width - 1.0f);

	// Refine by the short-range offsets.
	channels[0] = target_id;
	channels[1] = NUM_KEYPOINTS + target_id;
	for (int i = 0; i < offset_refinement_steps; ++i)
	{
		offsets = sample_tensor_at_multiple_channels(short_offsets, Point { y, x }, channels, 2 * NUM_KEYPOINTS);
		y = std::clamp(y + offsets[0], 0.0f, MAP_SIZE.height - 1.0f);
		x = std::clamp(x + offsets[1], 0.0f, MAP_SIZE.width - 1.0f);
	}

	return Point { y, x };
}

void backtrack_decode_pose(const std::vector<float> &scores, const std::vector<float> &short_offsets,
						   const std::vector<float> &mid_offsets, const KeypointWithScore &root,
						   const AdjacencyList &adjacency_list, PoseKeypoints &pose_keypoints,
						   PoseKeypointScores &keypoint_scores, unsigned int offset_refinement_steps)
{
	const float root_score = sample_tensor_at_single_channel(scores, root.point, root.id, NUM_KEYPOINTS);

	// Used in order to put candidate keypoints in a priority queue w.r.t. their
	// score. Keypoints with higher score have higher priority and will be
	// decoded/processed first.
	KeypointQueue decode_queue;
	decode_queue.push(KeypointWithScore(root.point, root.id, root_score));

	// Keeps track of the keypoints whose position has already been decoded.
	std::vector<bool> keypoint_decoded(NUM_KEYPOINTS, false);

	while (!decode_queue.empty())
	{
		// The top element in the queue is the next keypoint to be processed.
		const KeypointWithScore current_keypoint = decode_queue.top();
		decode_queue.pop();

		if (keypoint_decoded[current_keypoint.id])
			continue;

		pose_keypoints[current_keypoint.id] = current_keypoint.point;
		keypoint_scores[current_keypoint.id] = current_keypoint.score;

		keypoint_decoded[current_keypoint.id] = true;

		// Add the children of the current keypoint that have not been decoded yet
		// to the priority queue.
		const int num_children = adjacency_list.child_ids[current_keypoint.id].size();
		for (int j = 0; j < num_children; ++j)
		{
			const int child_id = adjacency_list.child_ids[current_keypoint.id][j];
			int edge_id = adjacency_list.edge_ids[current_keypoint.id][j];
			if (keypoint_decoded[child_id])
				continue;

			// The mid-offsets block is organized as 4 blocks of NUM_EDGES:
			// [fwd Y offsets][fwd X offsets][bwd Y offsets][bwd X offsets]
			// OTOH edge_id is [0,NUM_EDGES) for forward edges and
			// [NUM_EDGES, 2*NUM_EDGES) for backward edges.
			// Thus if the edge is a backward edge (>NUM_EDGES) then we need
			// to start 16 indices later to be correctly aligned with the mid-offsets.
			if (edge_id > NUM_EDGES)
				edge_id += NUM_EDGES;

			const Point child_point = find_displaced_position(short_offsets, mid_offsets, current_keypoint.point,
															  edge_id, child_id, offset_refinement_steps);
			const float child_score = sample_tensor_at_single_channel(scores, child_point, NUM_KEYPOINTS, child_id);
			decode_queue.emplace(child_point, child_id, child_score);
		}
	}
}

void find_overlapping_keypoints(std::vector<bool> &mask, const PoseKeypoints &pose1, const PoseKeypoints &pose2,
								const float squared_radius)
{
	for (unsigned int k = 0; k < mask.size(); ++k)
	{
		if (squared_distance(pose1[k], pose2[k]) <= squared_radius)
			mask[k] = true;
	}
}

void perform_soft_keypoint_NMS(std::vector<float> &all_instance_scores, const std::vector<int> &decreasing_indices,
							   const std::vector<PoseKeypoints> &all_keypoint_coords,
							   const std::vector<PoseKeypointScores> &all_keypoint_scores,
							   const float squared_nms_radius)
{
	const int num_instances = decreasing_indices.size();

	all_instance_scores.resize(num_instances);
	// Indicates the occlusion status of the keypoints of the active instance.
	std::vector<bool> keypoint_occluded(NUM_KEYPOINTS);
	// Indices of the keypoints of the active instance in decreasing score value.
	std::vector<int> indices(NUM_KEYPOINTS);
	for (int i = 0; i < num_instances; ++i)
	{
		const int current_index = decreasing_indices[i];
		// Find the keypoints of the current instance which are overlapping with
		// the corresponding keypoints of the higher-scoring instances and
		// zero-out their contribution to the score of the current instance.
		std::fill(keypoint_occluded.begin(), keypoint_occluded.end(), false);
		for (int j = 0; j < i; ++j)
		{
			const int previous_index = decreasing_indices[j];
			find_overlapping_keypoints(keypoint_occluded, all_keypoint_coords[current_index],
									   all_keypoint_coords[previous_index], squared_nms_radius);
		}
		// We compute the argsort keypoint indices based on the original keypoint
		// scores, but we do not let them contribute to the instance score if they
		// have been non-maximum suppressed.
		decreasing_arg_sort(indices, all_keypoint_scores[current_index].data(),
							all_keypoint_scores[current_index].size());
		float total_score = 0.0f;
		for (unsigned int k = 0; k < NUM_KEYPOINTS; ++k)
		{
			if (!keypoint_occluded[indices[k]])
				total_score += all_keypoint_scores[current_index][indices[k]];
		}
		all_instance_scores[current_index] = total_score / NUM_KEYPOINTS;
	}
}

} // namespace

#define NAME "imx500_posenet"

class PoseNet : public IMX500PostProcessingStage
{
public:
	PoseNet(RPiCamApp *app) : IMX500PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	std::vector<PoseResults> decodeAllPoses(const std::vector<float> &scores, const std::vector<float> &short_offsets,
											const std::vector<float> &mid_offsets);
	void translateCoordinates(std::vector<PoseResults> &results, const Rectangle &scaler_crop) const;
	void filterOutputObjects(const std::vector<PoseResults> &results);

	struct LtResults
	{
		PoseResults results;
		unsigned int visible;
		unsigned int hidden;
		bool matched;
	};

	std::vector<LtResults> lt_results_;
	std::mutex lt_lock_;

	// Config params:
	float threshold_;
	unsigned int max_detections_;
	unsigned int offset_refinement_steps_;
	float nms_radius_;
	bool temporal_filtering_;

	float tolerance_;
	float factor_;
	unsigned int visible_frames_;
	unsigned int hidden_frames_;
	bool started_ = false;
};

char const *PoseNet::Name() const
{
	return NAME;
}

void PoseNet::Read(boost::property_tree::ptree const &params)
{
	max_detections_ = params.get<unsigned int>("max_detections", 10);
	threshold_ = params.get<float>("threshold", 0.5f);
	offset_refinement_steps_ = params.get<unsigned int>("offset_refinement_steps", 5);
	nms_radius_ = params.get<float>("nms_radius", 10) / STRIDE;

	if (params.find("temporal_filter") != params.not_found())
	{
		temporal_filtering_ = true;
		tolerance_ = params.get<float>("temporal_filter.tolerance", 0.05);
		factor_ = params.get<float>("temporal_filter.factor", 0.2);
		visible_frames_ = params.get<unsigned int>("temporal_filter.visible_frames", 5);
		hidden_frames_ = params.get<unsigned int>("temporal_filter.hidden_frames", 2);
	}
	else
		temporal_filtering_ = false;

	IMX500PostProcessingStage::Read(params);
}

void PoseNet::Configure()
{
	lt_results_.clear();
	IMX500PostProcessingStage::Configure();
	if (!started_)
	{
		IMX500PostProcessingStage::ShowFwProgressBar();
		started_ = true;
	}
}

bool PoseNet::Process(CompletedRequestPtr &completed_request)
{
	auto scaler_crop = completed_request->metadata.get(controls::ScalerCrop);
	if (!raw_stream_ || !scaler_crop)
	{
		LOG_ERROR("Must have RAW stream and scaler crop available to get sensor dimensions!");
		return false;
	}

	auto output = completed_request->metadata.get(controls::rpi::CnnOutputTensor);
	if (!output)
	{
		LOG_ERROR("No output tensor found in metadata!");
		return false;
	}

	if (output->size() < NUM_HEATMAPS + NUM_SHORT_OFFSETS + NUM_MID_OFFSETS)
	{
		LOG_ERROR("Unexpected output tensor size: " << output->size());
		return false;
	}

	std::vector<float> scores =
		format_tensor((float *)output->data(), NUM_HEATMAPS / (MAP_SIZE.width * MAP_SIZE.height), 1);
	std::vector<float> short_offsets = format_tensor((float *)output->data() + NUM_HEATMAPS,
													 NUM_SHORT_OFFSETS / (MAP_SIZE.width * MAP_SIZE.height), STRIDE);
	std::vector<float> mid_offsets = format_tensor((float *)output->data() + NUM_HEATMAPS + NUM_SHORT_OFFSETS,
												   NUM_MID_OFFSETS / (MAP_SIZE.width * MAP_SIZE.height), STRIDE);

	std::vector<PoseResults> results = decodeAllPoses(scores, short_offsets, mid_offsets);
	translateCoordinates(results, *scaler_crop);

	std::vector<std::vector<libcamera::Point>> locations;
	std::vector<std::vector<float>> confidences;

	if (temporal_filtering_)
	{
		// Process() can be concurrently called through different threads for consecutive CompletedRequests if
		// things are running behind.  So protect access to the lt_results_ state object.
		std::scoped_lock<std::mutex> l(lt_lock_);

		filterOutputObjects(results);
		for (auto const &lt_r : lt_results_)
		{
			if (lt_r.hidden)
				continue;

			locations.push_back({});
			confidences.push_back({});

			for (auto const &s : lt_r.results.pose_keypoint_scores)
				confidences.back().push_back(s);
			for (auto const &k : lt_r.results.pose_keypoints)
				locations.back().emplace_back(k.x, k.y);
		}
	}
	else if (results.size())
	{
		for (auto const &result : results)
		{
			locations.push_back({});
			confidences.push_back({});

			for (auto const &s : result.pose_keypoint_scores)
				confidences.back().push_back(s);
			for (auto const &k : result.pose_keypoints)
				locations.back().emplace_back(k.x, k.y);
		}
	}

	if (locations.size() && locations.size() == confidences.size())
	{
		completed_request->post_process_metadata.Set("pose_estimation.locations", locations);
		completed_request->post_process_metadata.Set("pose_estimation.confidences", confidences);
	}

	return IMX500PostProcessingStage::Process(completed_request);
}

// Decodes poses from the score map, the short and mid offsets.
// "Block space" refers to the output y and z size of the network.
// For example if the network that takes a (353,481) (y,x) input image will have
// an output field of 23, 31. Thus the sizes of the input vectors to this
// functions will be
//   scores: 23 x 31 x NUM_KEYPOINTS
//   short_offsets 23 x 31 x 2 x NUM_KEYPOINTS (x and y per keypoint)
//   mid_offsets 23 x 31 x 2 x 2 2 NUM_EDGES (x and y for each fwd and bwd edge)
// Thus height and width need to be set to 23 and 31 respectively.
// nms_radius must also be given in these units.
// The output coordinates will be in pixel coordinates.
//
// For details see https://arxiv.org/abs/1803.08225
// PersonLab: Person Pose Estimation and Instance Segmentation with a
// Bottom-Up, Part-Based, Geometric Embedding Model
// George Papandreou, Tyler Zhu, Liang-Chieh Chen, Spyros Gidaris,
// Jonathan Tompson, Kevin Murphy
std::vector<PoseResults> PoseNet::decodeAllPoses(const std::vector<float> &scores,
												 const std::vector<float> &short_offsets,
												 const std::vector<float> &mid_offsets)
{
	const float min_score_logit = log_odds(threshold_);

	KeypointQueue queue = build_keypoint_queue(scores, short_offsets, min_score_logit);
	AdjacencyList adjacency_list = build_agency_list();

	std::vector<int> indices(NUM_KEYPOINTS);

	// Generate at most max_detections object instances per image in decreasing
	// root part score order.
	std::vector<float> all_instance_scores;

	std::vector<PoseKeypoints> scratch_poses(NUM_KEYPOINTS);
	std::vector<PoseKeypointScores> scratch_keypoint_scores(max_detections_);

	unsigned int pose_counter = 0;
	while (pose_counter < max_detections_ && !queue.empty())
	{
		// The top element in the queue is the next root candidate.
		const KeypointWithScore root = queue.top();
		queue.pop();

		// Reject a root candidate if it is within a disk of nms_radius_ pixels
		// from the corresponding part of a previously detected instance.
		if (!pass_keypoint_nms(scratch_poses, pose_counter, root, nms_radius_ * nms_radius_))
			continue;

		auto &next_pose = scratch_poses[pose_counter];
		auto &next_scores = scratch_keypoint_scores[pose_counter];
		for (unsigned int k = 0; k < NUM_KEYPOINTS; ++k)
		{
			next_pose[k].x = -1.0f;
			next_pose[k].y = -1.0f;
			next_scores[k] = -1E5;
		}

		backtrack_decode_pose(scores, short_offsets, mid_offsets, root, adjacency_list, next_pose, next_scores,
							  offset_refinement_steps_);

		// Convert keypoint-level scores from log-odds to probabilities and compute
		// an initial instance-level score as the average of the scores of the top-k
		// scoring keypoints.
		for (unsigned int k = 0; k < NUM_KEYPOINTS; ++k)
			next_scores[k] = sigmoid(next_scores[k]);

		decreasing_arg_sort(indices, next_scores.data(), next_scores.size());

		float instance_score = 0.0f;
		for (unsigned int j = 0; j < NUM_KEYPOINTS; ++j)
			instance_score += next_scores[indices[j]];

		instance_score /= NUM_KEYPOINTS;

		if (instance_score >= threshold_)
		{
			pose_counter++;
			all_instance_scores.push_back(instance_score);
		}
	}

	// Sort the detections in decreasing order of their instance-level scores.
	std::vector<int> decreasing_indices;
	decreasing_arg_sort(decreasing_indices, all_instance_scores);

	// Keypoint-level soft non-maximum suppression and instance-level rescoring as
	// the average of the top-k keypoints in terms of their keypoint-level scores.
	perform_soft_keypoint_NMS(all_instance_scores, decreasing_indices, scratch_poses, scratch_keypoint_scores,
							  nms_radius_ * nms_radius_);

	// Sort the detections in decreasing order of their final instance-level
	// scores. Usually the order does not change but this is not guaranteed.
	decreasing_arg_sort(decreasing_indices, all_instance_scores);

	std::vector<PoseResults> results;
	for (int index : decreasing_indices)
	{
		if (all_instance_scores[index] < threshold_)
			break;

		// New result.
		results.push_back({});

		// Rescale keypoint coordinates into pixel space (much more useful for user).
		for (unsigned int k = 0; k < NUM_KEYPOINTS; ++k)
		{
			results.back().pose_keypoints[k].y = scratch_poses[index][k].y * STRIDE;
			results.back().pose_keypoints[k].x = scratch_poses[index][k].x * STRIDE;
		}

		std::copy(scratch_keypoint_scores[index].begin(), scratch_keypoint_scores[index].end(),
				  results.back().pose_keypoint_scores.begin());
		results.back().pose_score = all_instance_scores[index];
	}

	return results;
}

void PoseNet::translateCoordinates(std::vector<PoseResults> &results, const Rectangle &scaler_crop) const
{
	for (auto &r : results)
	{
		for (auto &keypoint : r.pose_keypoints)
		{
			std::vector<float> coords{ keypoint.x / (INPUT_TENSOR_SIZE.width - 1),
									   keypoint.y / (INPUT_TENSOR_SIZE.height - 1),
									   0, 0 };
			Rectangle translated = ConvertInferenceCoordinates(coords, scaler_crop);
			keypoint.x = translated.x;
			keypoint.y = translated.y;
		}
	}
}

void PoseNet::filterOutputObjects(const std::vector<PoseResults> &results)
{
	const Size isp_output_size = output_stream_->configuration().size;

	for (auto &lt_r : lt_results_)
		lt_r.matched = false;

	for (auto const &r : results)
	{
		bool matched = false;
		for (auto &lt_r : lt_results_)
		{
			// Try and match a detected object in our long term list.
			bool found = true;
			for (unsigned int i = 0; i < NUM_KEYPOINTS; i++)
			{
				if (std::abs(lt_r.results.pose_keypoints[i].x - r.pose_keypoints[i].x) >
						tolerance_ * isp_output_size.width ||
					std::abs(lt_r.results.pose_keypoints[i].y - r.pose_keypoints[i].y) >
						tolerance_ * isp_output_size.height)
				{
					found = false;
					break;
				}
			}

			if (found)
			{
				lt_r.matched = matched = true;
				lt_r.results.pose_score = r.pose_score;

				for (unsigned int i = 0; i < NUM_KEYPOINTS; i++)
				{
					lt_r.results.pose_keypoint_scores[i] =
						factor_ * r.pose_keypoint_scores[i] + (1 - factor_) * lt_r.results.pose_keypoint_scores[i];
					lt_r.results.pose_keypoints[i].x =
						factor_ * r.pose_keypoints[i].x + (1 - factor_) * lt_r.results.pose_keypoints[i].x;
					lt_r.results.pose_keypoints[i].y =
						factor_ * r.pose_keypoints[i].y + (1 - factor_) * lt_r.results.pose_keypoints[i].y;
				}

				// Reset the visibility counter for when the result next disappears.
				lt_r.visible = visible_frames_;
				// Decrement the hidden counter until the result becomes visible in the list.
				lt_r.hidden = std::max(0, (int)lt_r.hidden - 1);
				break;
			}
		}

		// Add the result to the long term list if not found.  This result will remain hidden for hidden_frames_
		// consecutive frames.
		if (!matched)
			lt_results_.push_back({ r, visible_frames_, hidden_frames_, true });
	}

	for (auto &lt_r : lt_results_)
	{
		if (!lt_r.matched)
		{
			// If a non matched result in the long term list is still hidden, set visible count to 0 so that it must be
			// matched for hidden_frames_ consecutive frames before becoming visible. Otherwise, decrement the visible
			// count of unmatched objects in the long term list.
			if (lt_r.hidden)
				lt_r.visible = 0;
			else
				lt_r.visible--;
		}
	}

	// Remove now invisible objects from the long term list.
	lt_results_.erase(std::remove_if(lt_results_.begin(), lt_results_.end(),
						[] (const LtResults &obj) { return !obj.matched && !obj.visible; }),
					  lt_results_.end());
}

PostProcessingStage *Create(RPiCamApp *app)
{
	return new PoseNet(app);
}

RegisterStage reg(NAME, &Create);
