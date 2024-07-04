/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * libav_encoder.cpp - libav video encoder.
 */

#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <libdrm/drm_fourcc.h>
#include <linux/videodev2.h>

#include <chrono>
#include <iostream>

#include "libav_encoder.hpp"

namespace {

void encoderOptionsGeneral(VideoOptions const *options, AVCodecContext *codec)
{
	codec->framerate = { (int)(options->framerate.value_or(DEFAULT_FRAMERATE) * 1000), 1000 };
	codec->profile = FF_PROFILE_UNKNOWN;

	if (!options->profile.empty())
	{
		const AVCodecDescriptor *desc = avcodec_descriptor_get(codec->codec_id);
		for (const AVProfile *profile = desc->profiles; profile && profile->profile != FF_PROFILE_UNKNOWN; profile++)
		{
			if (!strncasecmp(options->profile.c_str(), profile->name, options->profile.size()))
			{
				codec->profile = profile->profile;
				break;
			}
		}
		if (codec->profile == FF_PROFILE_UNKNOWN)
			throw std::runtime_error("libav: no such profile " + options->profile);
	}

	codec->level = options->level.empty() ? FF_LEVEL_UNKNOWN : std::stof(options->level) * 10;
	codec->gop_size = options->intra ? options->intra : (int)(options->framerate.value_or(DEFAULT_FRAMERATE));

	if (options->bitrate)
		codec->bit_rate = options->bitrate.bps();

	if (!options->libav_video_codec_opts.empty())
	{
		const std::string &opts = options->libav_video_codec_opts;
		for (std::string::size_type i = 0, n = 0; i != std::string::npos; i = n)
		{
			n = opts.find(';', i);
			const std::string opt = opts.substr(i, n - i);
			if (n != std::string::npos)
				n++;
			if (opt.empty())
				continue;
			std::string::size_type kn = opt.find('=');
			const std::string key = opt.substr(0, kn);
			const std::string value = (kn != std::string::npos) ? opt.substr(kn + 1) : "";
			int ret = av_opt_set(codec, key.c_str(), value.c_str(), AV_OPT_SEARCH_CHILDREN);
			if (ret < 0)
			{
				char err[AV_ERROR_MAX_STRING_SIZE];
				av_strerror(ret, err, sizeof(err));
				throw std::runtime_error("libav: codec option error " + opt + ": " + err);
			}
		}
	}
}

void encoderOptionsH264M2M(VideoOptions const *options, AVCodecContext *codec)
{
	codec->pix_fmt = AV_PIX_FMT_DRM_PRIME;
	codec->max_b_frames = 0;
}

void encoderOptionsLibx264(VideoOptions const *options, AVCodecContext *codec)
{
	codec->max_b_frames = 1;
	codec->me_range = 16;
	codec->me_cmp = 1; // No chroma ME
	codec->me_subpel_quality = 0;
	codec->thread_count = 0;
	codec->thread_type = FF_THREAD_FRAME;
	codec->slices = 1;

	av_opt_set(codec->priv_data, "preset", "superfast", 0);
	av_opt_set(codec->priv_data, "partitions", "i8x8,i4x4", 0);
	av_opt_set(codec->priv_data, "weightp", "none", 0);
	av_opt_set(codec->priv_data, "weightb", "0", 0);
	av_opt_set(codec->priv_data, "motion-est", "dia", 0);
	av_opt_set(codec->priv_data, "sc_threshold", "0", 0);
	av_opt_set(codec->priv_data, "rc-lookahead", "0", 0);
	av_opt_set(codec->priv_data, "mixed_ref", "0", 0);
}

const std::map<std::string, std::function<void(VideoOptions const *, AVCodecContext *)>> optionsMap =
{
	{ "h264_v4l2m2m", encoderOptionsH264M2M },
	{ "libx264", encoderOptionsLibx264 },
};

} // namespace

void LibAvEncoder::initVideoCodec(VideoOptions const *options, StreamInfo const &info)
{
	const AVCodec *codec = avcodec_find_encoder_by_name(options->libav_video_codec.c_str());
	if (!codec)
		throw std::runtime_error("libav: cannot find video encoder " + options->libav_video_codec);

	codec_ctx_[Video] = avcodec_alloc_context3(codec);
	if (!codec_ctx_[Video])
		throw std::runtime_error("libav: Cannot allocate video context");

	codec_ctx_[Video]->width = info.width;
	codec_ctx_[Video]->height = info.height;
	// usec timebase
	codec_ctx_[Video]->time_base = { 1, 1000 * 1000 };
	codec_ctx_[Video]->sw_pix_fmt = AV_PIX_FMT_YUV420P;
	codec_ctx_[Video]->pix_fmt = AV_PIX_FMT_YUV420P;

	if (info.colour_space)
	{
		using libcamera::ColorSpace;

		static const std::map<ColorSpace::Primaries, AVColorPrimaries> pri_map = {
			{ ColorSpace::Primaries::Smpte170m, AVCOL_PRI_SMPTE170M },
			{ ColorSpace::Primaries::Rec709, AVCOL_PRI_BT709 },
			{ ColorSpace::Primaries::Rec2020, AVCOL_PRI_BT2020 },
		};

		static const std::map<ColorSpace::TransferFunction, AVColorTransferCharacteristic> tf_map = {
			{ ColorSpace::TransferFunction::Linear, AVCOL_TRC_LINEAR },
			{ ColorSpace::TransferFunction::Srgb, AVCOL_TRC_IEC61966_2_1 },
			{ ColorSpace::TransferFunction::Rec709, AVCOL_TRC_BT709 },
		};

		static const std::map<ColorSpace::YcbcrEncoding, AVColorSpace> cs_map = {
			{ ColorSpace::YcbcrEncoding::None, AVCOL_SPC_UNSPECIFIED },
			{ ColorSpace::YcbcrEncoding::Rec601, AVCOL_SPC_SMPTE170M },
			{ ColorSpace::YcbcrEncoding::Rec709, AVCOL_SPC_BT709 },
			{ ColorSpace::YcbcrEncoding::Rec2020, AVCOL_SPC_BT2020_CL },
		};

		auto it_p = pri_map.find(info.colour_space->primaries);
		if (it_p == pri_map.end())
			throw std::runtime_error("libav: no match for colour primaries in " + info.colour_space->toString());
		codec_ctx_[Video]->color_primaries = it_p->second;

		auto it_tf = tf_map.find(info.colour_space->transferFunction);
		if (it_tf == tf_map.end())
			throw std::runtime_error("libav: no match for transfer function in " + info.colour_space->toString());
		codec_ctx_[Video]->color_trc = it_tf->second;

		auto it_cs = cs_map.find(info.colour_space->ycbcrEncoding);
		if (it_cs == cs_map.end())
			throw std::runtime_error("libav: no match for ycbcr encoding in " + info.colour_space->toString());
		codec_ctx_[Video]->colorspace = it_cs->second;

		codec_ctx_[Video]->color_range =
			info.colour_space->range == ColorSpace::Range::Full ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
	}

	// Apply any codec specific options:
	auto fn = optionsMap.find(options->libav_video_codec);
	if (fn != optionsMap.end())
		fn->second(options, codec_ctx_[Video]);

	// Apply general options.
	encoderOptionsGeneral(options, codec_ctx_[Video]);

	const std::string tcp { "tcp://" };
	const std::string udp { "udp://" };

	// Setup an appropriate stream/container format.
	const char *format = nullptr;
	if (options->libav_format.empty())
	{
		// Check if output_file_ starts with a "tcp://" or "udp://" url.
		// C++ 20 has a convenient starts_with() function for this which we may eventually use.		
		if (output_file_.empty() ||
			output_file_.find(tcp.c_str(), 0, tcp.length()) != std::string::npos ||
			output_file_.find(udp.c_str(), 0, udp.length()) != std::string::npos)
		{
			if (options->libav_video_codec == "h264_v4l2m2m" || options->libav_video_codec == "libx264")
				format = "h264";
			else
				throw std::runtime_error("libav: please specify output format with the --libav-format argument");
		}
	}
	else
		format = options->libav_format.c_str();

	// Legacy handling of the --listen parameter.  If missing from the url string, add "?listen=1" to the end.
	if (options->listen && output_file_.find(tcp.c_str(), 0, tcp.length()) != std::string::npos)
	{
		const std::string listen { "?listen=1" };
		// Check if output_file_ ends with "?listen=1" parameter.
		// C++ 20 has a convenient ends_with() function for this which we may eventually use.
		if (output_file_.find(listen, output_file_.length() - listen.length()) == std::string::npos)
			output_file_ += listen;
	}

	assert(out_fmt_ctx_ == nullptr);
	avformat_alloc_output_context2(&out_fmt_ctx_, nullptr, format, output_file_.c_str());
	if (!out_fmt_ctx_)
		throw std::runtime_error("libav: cannot allocate output context, try setting with --libav-format");

	if (out_fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER)
		codec_ctx_[Video]->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

	int ret = avcodec_open2(codec_ctx_[Video], codec, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open video codec: " + std::to_string(ret));

	// Video stream
	stream_[Video] = avformat_new_stream(out_fmt_ctx_, codec);
	if (!stream_[Video])
		throw std::runtime_error("libav: cannot allocate stream for vidout output context");

	// The avi stream context seems to need the video stream time_base set to
	// 1/framerate to report the correct framerate in the container file.
	//
	// This seems to be a limitation/bug in ffmpeg:
	// https://github.com/FFmpeg/FFmpeg/blob/3141dbb7adf1e2bd5b9ff700312d7732c958b8df/libavformat/avienc.c#L527
	if (!strncmp(out_fmt_ctx_->oformat->name, "avi", 3))
		stream_[Video]->time_base = { 1000, (int)(options->framerate.value_or(DEFAULT_FRAMERATE) * 1000) };
	else
		stream_[Video]->time_base = codec_ctx_[Video]->time_base;

	stream_[Video]->avg_frame_rate = stream_[Video]->r_frame_rate = codec_ctx_[Video]->framerate;
	avcodec_parameters_from_context(stream_[Video]->codecpar, codec_ctx_[Video]);
}

void LibAvEncoder::initAudioInCodec(VideoOptions const *options, StreamInfo const &info)
{
#if LIBAVUTIL_VERSION_MAJOR < 58
	AVInputFormat *input_fmt = (AVInputFormat *)av_find_input_format(options->audio_source.c_str());
#else
	const AVInputFormat *input_fmt = (AVInputFormat *)av_find_input_format(options->audio_source.c_str());
#endif

	assert(in_fmt_ctx_ == nullptr);

	int ret;
	AVDictionary *format_opts = nullptr;

	if (options->audio_channels != 0)
		ret = av_dict_set_int(&format_opts, "channels", options->audio_channels, 0);

	ret = avformat_open_input(&in_fmt_ctx_, options->audio_device.c_str(), input_fmt, &format_opts);
	if (ret < 0)
	{
		av_dict_free(&format_opts);
		throw std::runtime_error("libav: cannot open " + options->audio_source + " input device " + options->audio_device);
	}

	av_dict_free(&format_opts);

	avformat_find_stream_info(in_fmt_ctx_, nullptr);

	stream_[AudioIn] = nullptr;
	for (unsigned int i = 0; i < in_fmt_ctx_->nb_streams; i++)
	{
		if (in_fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
		{
			stream_[AudioIn] = in_fmt_ctx_->streams[i];
			break;
		}
	}

	if (!stream_[AudioIn])
		throw std::runtime_error("libav: couldn't find a audio stream.");

	const AVCodec *codec = avcodec_find_decoder(stream_[AudioIn]->codecpar->codec_id);
	codec_ctx_[AudioIn] = avcodec_alloc_context3(codec);
	avcodec_parameters_to_context(codec_ctx_[AudioIn], stream_[AudioIn]->codecpar);
	// usec timebase
	codec_ctx_[AudioIn]->time_base = { 1, 1000 * 1000 };
	ret = avcodec_open2(codec_ctx_[AudioIn], codec, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open audio in codec: " + std::to_string(ret));
}

void LibAvEncoder::initAudioOutCodec(VideoOptions const *options, StreamInfo const &info)
{
	const AVCodec *codec = avcodec_find_encoder_by_name(options->audio_codec.c_str());
	if (!codec)
		throw std::runtime_error("libav: cannot find audio encoder " + options->audio_codec);

	codec_ctx_[AudioOut] = avcodec_alloc_context3(codec);
	if (!codec_ctx_[AudioOut])
		throw std::runtime_error("libav: cannot allocate audio in context");

	assert(stream_[AudioIn]);

#if LIBAVUTIL_VERSION_MAJOR < 57
	codec_ctx_[AudioOut]->channels = stream_[AudioIn]->codecpar->channels;
	codec_ctx_[AudioOut]->channel_layout = av_get_default_channel_layout(stream_[AudioIn]->codecpar->channels);
#else
	av_channel_layout_default(&codec_ctx_[AudioOut]->ch_layout, stream_[AudioIn]->codecpar->ch_layout.nb_channels);
#endif

	codec_ctx_[AudioOut]->sample_rate = options->audio_samplerate ? options->audio_samplerate
																  : stream_[AudioIn]->codecpar->sample_rate;
	codec_ctx_[AudioOut]->sample_fmt = codec->sample_fmts[0];
	codec_ctx_[AudioOut]->bit_rate = options->audio_bitrate.bps();
	// usec timebase
	codec_ctx_[AudioOut]->time_base = { 1, 1000 * 1000 };

	assert(out_fmt_ctx_);
	if (out_fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER)
		codec_ctx_[AudioOut]->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

	int ret = avcodec_open2(codec_ctx_[AudioOut], codec, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open audio codec: " + std::to_string(ret));

	stream_[AudioOut] = avformat_new_stream(out_fmt_ctx_, codec);
	if (!stream_[AudioOut])
		throw std::runtime_error("libav: cannot allocate stream for audio output context");

	stream_[AudioOut]->time_base = codec_ctx_[AudioOut]->time_base;
	avcodec_parameters_from_context(stream_[AudioOut]->codecpar, codec_ctx_[AudioOut]);
}

LibAvEncoder::LibAvEncoder(VideoOptions const *options, StreamInfo const &info)
	: Encoder(options), output_ready_(false), abort_video_(false), abort_audio_(false), video_start_ts_(0),
	  audio_samples_(0), in_fmt_ctx_(nullptr), out_fmt_ctx_(nullptr), output_file_(options->output)
{
	if (options->circular || options->segment || !options->save_pts.empty() || options->split)
		LOG_ERROR("\nERROR: Pi 5 and libav encoder does not currently support the circular, segment, save_pts or "
				  "split command line options, they will be ignored!\n");

	avdevice_register_all();

	if (options->verbose >= 2)
		av_log_set_level(AV_LOG_VERBOSE);

	initVideoCodec(options, info);
	if (options->libav_audio)
	{
		initAudioInCodec(options, info);
		initAudioOutCodec(options, info);
		av_dump_format(in_fmt_ctx_, 0, options_->audio_device.c_str(), 0);
	}

	av_dump_format(out_fmt_ctx_, 0, output_file_.c_str(), 1);

	LOG(2, "libav: codec init completed");

	video_thread_ = std::thread(&LibAvEncoder::videoThread, this);

	if (options->libav_audio)
		audio_thread_ = std::thread(&LibAvEncoder::audioThread, this);
}

LibAvEncoder::~LibAvEncoder()
{
	if (options_->libav_audio)
	{
		abort_audio_ = true;
		audio_thread_.join();
	}

	abort_video_ = true;
	video_thread_.join();

	avformat_free_context(out_fmt_ctx_);
	avcodec_free_context(&codec_ctx_[Video]);

	if (options_->libav_audio)
	{
		avformat_free_context(in_fmt_ctx_);
		avcodec_free_context(&codec_ctx_[AudioIn]);
		avcodec_free_context(&codec_ctx_[AudioOut]);
	}

	LOG(2, "libav: codec closed");
}

void LibAvEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
	AVFrame *frame = av_frame_alloc();
	if (!frame)
		throw std::runtime_error("libav: could not allocate AVFrame");

	if (!video_start_ts_)
		video_start_ts_ = timestamp_us;

	frame->format = codec_ctx_[Video]->pix_fmt;
	frame->width = info.width;
	frame->height = info.height;
	frame->linesize[0] = info.stride;
	frame->linesize[1] = frame->linesize[2] = info.stride >> 1;
	frame->pts = timestamp_us - video_start_ts_ +
				 (options_->av_sync.value < 0us ? -options_->av_sync.get<std::chrono::microseconds>() : 0);

	if (codec_ctx_[Video]->pix_fmt == AV_PIX_FMT_DRM_PRIME)
	{
		std::scoped_lock<std::mutex> lock(drm_queue_lock_);
		drm_frame_queue_.emplace(std::make_unique<AVDRMFrameDescriptor>());
		frame->buf[0] = av_buffer_create((uint8_t *)drm_frame_queue_.back().get(), sizeof(AVDRMFrameDescriptor),
										 &LibAvEncoder::releaseBuffer, this, 0);
		frame->data[0] = frame->buf[0]->data;

		AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)frame->data[0];
		desc->nb_objects = 1;
		desc->objects[0].fd = fd;
		desc->objects[0].size = size;
		desc->objects[0].format_modifier = DRM_FORMAT_MOD_INVALID;

		desc->nb_layers = 1;
		desc->layers[0].format = DRM_FORMAT_YUV420;
		desc->layers[0].nb_planes = 3;
		desc->layers[0].planes[0].object_index = 0;
		desc->layers[0].planes[0].offset = 0;
		desc->layers[0].planes[0].pitch = info.stride;
		desc->layers[0].planes[1].object_index = 0;
		desc->layers[0].planes[1].offset = info.stride * info.height;
		desc->layers[0].planes[1].pitch = info.stride >> 1;
		desc->layers[0].planes[2].object_index = 0;
		desc->layers[0].planes[2].offset = info.stride * info.height * 5 / 4;
		desc->layers[0].planes[2].pitch = info.stride >> 1;
	}
	else
	{
		frame->buf[0] = av_buffer_create((uint8_t *)mem, size, &LibAvEncoder::releaseBuffer, this, 0);
		av_image_fill_pointers(frame->data, AV_PIX_FMT_YUV420P, frame->height, frame->buf[0]->data, frame->linesize);
		av_frame_make_writable(frame);
	}

	std::scoped_lock<std::mutex> lock(video_mutex_);
	frame_queue_.push(frame);
	video_cv_.notify_all();
}

void LibAvEncoder::initOutput()
{
	int ret;

	// Copy the global header from the video encode context once the first frame
	// has been encoded.
	avcodec_parameters_from_context(stream_[Video]->codecpar, codec_ctx_[Video]);

	char err[64];
	if (!(out_fmt_ctx_->flags & AVFMT_NOFILE))
	{
		std::string filename = output_file_.empty() ? "/dev/null" : output_file_;

		// libav uses "pipe:" for stdout
		if (filename == "-")
			filename = std::string("pipe:");

		ret = avio_open2(&out_fmt_ctx_->pb, filename.c_str(), AVIO_FLAG_WRITE, nullptr, nullptr);
		if (ret < 0)
		{
			av_strerror(ret, err, sizeof(err));
			throw std::runtime_error("libav: unable to open output mux for " + output_file_ + ": " + err);
		}
	}

	ret = avformat_write_header(out_fmt_ctx_, nullptr);
	if (ret < 0)
	{
		av_strerror(ret, err, sizeof(err));
		throw std::runtime_error("libav: unable write output mux header for " + output_file_ + ": " + err);
	}
}

void LibAvEncoder::deinitOutput()
{
	if (!out_fmt_ctx_)
		return;

	av_write_trailer(out_fmt_ctx_);

	if (!(out_fmt_ctx_->flags & AVFMT_NOFILE))
		avio_closep(&out_fmt_ctx_->pb);
}

void LibAvEncoder::encode(AVPacket *pkt, unsigned int stream_id)
{
	int ret = 0;

	while (ret >= 0)
	{
		ret = avcodec_receive_packet(codec_ctx_[stream_id], pkt);

		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
		{
			av_packet_unref(pkt);
			break;
		}
		else if (ret < 0)
			throw std::runtime_error("libav: error receiving packet: " + std::to_string(ret));

		// Initialise the ouput mux on the first received video packet, as we may need
		// to copy global header data from the encoder.
		if (stream_id == Video && !output_ready_)
		{
			initOutput();
			output_ready_ = true;
		}

		pkt->stream_index = stream_id;
		pkt->pos = -1;
		pkt->duration = 0;

		// Rescale from the codec timebase to the stream timebase.
		av_packet_rescale_ts(pkt, codec_ctx_[stream_id]->time_base, out_fmt_ctx_->streams[stream_id]->time_base);

		std::scoped_lock<std::mutex> lock(output_mutex_);
		// pkt is now blank (av_interleaved_write_frame() takes ownership of
		// its contents and resets pkt), so that no unreferencing is necessary.
		// This would be different if one used av_write_frame().
		ret = av_interleaved_write_frame(out_fmt_ctx_, pkt);
		if (ret < 0)
		{
			char err[AV_ERROR_MAX_STRING_SIZE];
			av_strerror(ret, err, sizeof(err));
			throw std::runtime_error("libav: error writing output: " + std::string(err));
		}
	}
}

extern "C" void LibAvEncoder::releaseBuffer(void *opaque, uint8_t *data)
{
	LibAvEncoder *enc = static_cast<LibAvEncoder *>(opaque);

	enc->input_done_callback_(nullptr);

	// Pop the entry from the queue to release the AVDRMFrameDescriptor allocation
	std::scoped_lock<std::mutex> lock(enc->drm_queue_lock_);
	if (!enc->drm_frame_queue_.empty())
		enc->drm_frame_queue_.pop();
}

void LibAvEncoder::videoThread()
{
	AVPacket *pkt = av_packet_alloc();
	AVFrame *frame = nullptr;

	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(video_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				// Must check the abort first, to allow items in the output
				// queue to have a callback.
				if (abort_video_ && frame_queue_.empty())
					goto done;

				if (!frame_queue_.empty())
				{
					frame = frame_queue_.front();
					frame_queue_.pop();
					break;
				}
				else
					video_cv_.wait_for(lock, 200ms);
			}
		}

		int ret = avcodec_send_frame(codec_ctx_[Video], frame);
		if (ret < 0)
			throw std::runtime_error("libav: error encoding frame: " + std::to_string(ret));

		encode(pkt, Video);
		av_frame_free(&frame);
	}

done:
	// Flush the encoder
	avcodec_send_frame(codec_ctx_[Video], nullptr);
	encode(pkt, Video);

	av_packet_free(&pkt);
	deinitOutput();
}

void LibAvEncoder::audioThread()
{
	const AVSampleFormat required_fmt = codec_ctx_[AudioOut]->sample_fmt;
	// Amount of time to pre-record audio into the fifo before the first video frame.
	constexpr std::chrono::milliseconds pre_record_time(10);
	int ret;

#if LIBAVUTIL_VERSION_MAJOR < 57
	uint32_t out_channels = codec_ctx_[AudioOut]->channels;
#else
	uint32_t out_channels = codec_ctx_[AudioOut]->ch_layout.nb_channels;
#endif

	SwrContext *conv;
	AVAudioFifo *fifo;

#if LIBAVUTIL_VERSION_MAJOR < 57
	conv = swr_alloc_set_opts(nullptr, av_get_default_channel_layout(codec_ctx_[AudioOut]->channels), required_fmt,
							  stream_[AudioOut]->codecpar->sample_rate,
							  av_get_default_channel_layout(codec_ctx_[AudioIn]->channels),
							  codec_ctx_[AudioIn]->sample_fmt, codec_ctx_[AudioIn]->sample_rate, 0, nullptr);

	// 2 seconds FIFO buffer
	fifo = av_audio_fifo_alloc(required_fmt, codec_ctx_[AudioOut]->channels, codec_ctx_[AudioOut]->sample_rate * 2);
#else
	ret = swr_alloc_set_opts2(&conv, &codec_ctx_[AudioOut]->ch_layout, required_fmt,
							  stream_[AudioOut]->codecpar->sample_rate, &codec_ctx_[AudioIn]->ch_layout,
							  codec_ctx_[AudioIn]->sample_fmt, codec_ctx_[AudioIn]->sample_rate, 0, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: cannot create swr context");

	// 2 seconds FIFO buffer
	fifo = av_audio_fifo_alloc(required_fmt, codec_ctx_[AudioOut]->ch_layout.nb_channels,
							   codec_ctx_[AudioOut]->sample_rate * 2);
#endif

	swr_init(conv);

	AVPacket *in_pkt = av_packet_alloc();
	AVPacket *out_pkt = av_packet_alloc();
	AVFrame *in_frame = av_frame_alloc();
	uint8_t **samples = nullptr;
	int sample_linesize = 0;

	int max_output_samples = av_rescale_rnd(codec_ctx_[AudioOut]->frame_size, codec_ctx_[AudioOut]->sample_rate,
											codec_ctx_[AudioIn]->sample_rate, AV_ROUND_UP);
	ret = av_samples_alloc_array_and_samples(&samples, &sample_linesize, out_channels, max_output_samples, required_fmt,
											 0);
	if (ret < 0)
		throw std::runtime_error("libav: failed to alloc sample array");

	while (!abort_audio_)
	{
		// Audio In
		ret = av_read_frame(in_fmt_ctx_, in_pkt);
		if (ret < 0)
			throw std::runtime_error("libav: cannot read audio in frame");

		ret = avcodec_send_packet(codec_ctx_[AudioIn], in_pkt);
		if (ret < 0)
			throw std::runtime_error("libav: cannot send pkt for decoding audio in");

		ret = avcodec_receive_frame(codec_ctx_[AudioIn], in_frame);
		if (ret && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
			throw std::runtime_error("libav: error getting decoded audio in frame");

		// Audio Resample/Conversion
		int num_output_samples =
			av_rescale_rnd(swr_get_delay(conv, codec_ctx_[AudioIn]->sample_rate) + in_frame->nb_samples,
						   codec_ctx_[AudioOut]->sample_rate, codec_ctx_[AudioIn]->sample_rate, AV_ROUND_UP);

		if (num_output_samples > max_output_samples)
		{
			av_freep(&samples[0]);
			max_output_samples = num_output_samples;
			ret = av_samples_alloc_array_and_samples(&samples, &sample_linesize, out_channels, max_output_samples,
													 required_fmt, 0);
			if (ret < 0)
				throw std::runtime_error("libav: failed to alloc sample array");
		}

		ret = swr_convert(conv, samples, num_output_samples, (const uint8_t **)in_frame->extended_data,
						  in_frame->nb_samples);
		if (ret < 0)
			throw std::runtime_error("libav: swr_convert failed");

		// Pre-record some audio before the encoded video frame is available.
		if (!output_ready_)
		{
			using namespace std::chrono_literals;
			// Pre-record number of samples needed.
			const unsigned int ns = pre_record_time * stream_[AudioOut]->codecpar->sample_rate / 1s;
			unsigned int r = ns % codec_ctx_[AudioOut]->frame_size;
			// Number of pre-record samples rounded to the frame size.
			unsigned int ps = !r ? ns : ns + codec_ctx_[AudioOut]->frame_size - r;
			// FIFO size with samples from the next frame added.
			unsigned int fs = av_audio_fifo_size(fifo) + num_output_samples;
			if (fs > ps)
				av_audio_fifo_drain(fifo, fs - ps);
		}

		if (av_audio_fifo_space(fifo) < num_output_samples)
		{
			LOG(1, "libav: Draining audio fifo, configure a larger size");
			av_audio_fifo_drain(fifo, num_output_samples);
		}

		av_audio_fifo_write(fifo, (void **)samples, num_output_samples);

		av_frame_unref(in_frame);
		av_packet_unref(in_pkt);

		// Not yet ready to generate encoded audio!
		if (!output_ready_)
			continue;

		// Audio Out
		while (av_audio_fifo_size(fifo) >= codec_ctx_[AudioOut]->frame_size)
		{
			AVFrame *out_frame = av_frame_alloc();
			out_frame->nb_samples = codec_ctx_[AudioOut]->frame_size;

#if LIBAVUTIL_VERSION_MAJOR < 57
			out_frame->channels = codec_ctx_[AudioOut]->channels;
			out_frame->channel_layout = av_get_default_channel_layout(codec_ctx_[AudioOut]->channels);
#else
			av_channel_layout_copy(&out_frame->ch_layout, &codec_ctx_[AudioOut]->ch_layout);
#endif

			out_frame->format = required_fmt;
			out_frame->sample_rate = codec_ctx_[AudioOut]->sample_rate;

			av_frame_get_buffer(out_frame, 0);
			av_audio_fifo_read(fifo, (void **)out_frame->data, codec_ctx_[AudioOut]->frame_size);

			AVRational num = { 1, out_frame->sample_rate };
			int64_t ts = av_rescale_q(audio_samples_, num, codec_ctx_[AudioOut]->time_base);

			out_frame->pts = ts +
				(options_->av_sync.value > 0us ? options_->av_sync.get<std::chrono::microseconds>() : 0);
			audio_samples_ += codec_ctx_[AudioOut]->frame_size;

			ret = avcodec_send_frame(codec_ctx_[AudioOut], out_frame);
			if (ret < 0)
				throw std::runtime_error("libav: error encoding frame: " + std::to_string(ret));

			encode(out_pkt, AudioOut);
			av_frame_free(&out_frame);
		}
	}

	// Flush the encoder
	avcodec_send_frame(codec_ctx_[AudioOut], nullptr);
	encode(out_pkt, AudioOut);

	swr_free(&conv);
	av_freep(&samples[0]);
	av_audio_fifo_free(fifo);

	av_packet_free(&in_pkt);
	av_packet_free(&out_pkt);
	av_frame_free(&in_frame);
}
