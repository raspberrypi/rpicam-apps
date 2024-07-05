// recording_manager.hpp

#pragma once

#include <mutex>
#include <chrono>
#include <fstream>
#include <string>
#include <optional>

class RecordingManager {
public:
	static RecordingManager& getInstance() {
		static RecordingManager instance;
		return instance;
	}

	void objectDetected() {
		std::lock_guard<std::mutex> lock(mutex_);
		auto now = std::chrono::steady_clock::now();
		std::ofstream file(state_file_);
		file << now.time_since_epoch().count();
	}

	bool shouldRecord() {
		std::lock_guard<std::mutex> lock(mutex_);
		std::ifstream file(state_file_);
		if (!file) {
			return false;  // No object has been detected yet
		}
		std::chrono::steady_clock::duration::rep last_detection_count;
		file >> last_detection_count;
		if (file.fail()) {
			return false;  // Failed to read the file
		}
		auto last_detection = std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(last_detection_count));
		auto now = std::chrono::steady_clock::now();
		auto time_since_last_detection = std::chrono::duration_cast<std::chrono::seconds>(now - last_detection).count();
		return time_since_last_detection <= post_detection_record_time_;
	}

	void setPostDetectionRecordTime(int seconds) {
		std::lock_guard<std::mutex> lock(mutex_);
		post_detection_record_time_ = seconds;
	}

private:
	RecordingManager() : post_detection_record_time_(30), state_file_("/tmp/recording_state") {}

	RecordingManager(const RecordingManager&) = delete;
	RecordingManager& operator=(const RecordingManager&) = delete;

	int post_detection_record_time_; // in seconds
	std::mutex mutex_;
	std::string state_file_;
};
