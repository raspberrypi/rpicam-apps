// recording_manager.hpp

#pragma once

#include <mutex>
#include <chrono>
#include <optional>
#include <iostream>

class RecordingManager {
public:
	static RecordingManager& getInstance() {
		static RecordingManager instance;
		return instance;
	}

	void objectDetected() {
		std::lock_guard<std::mutex> lock(mutex_);
		std::cout << "Object detected" << std::endl;
		last_detection_time_ = std::chrono::steady_clock::now();
	}

	bool shouldRecord() {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!last_detection_time_) {
			return false;  // No object has been detected yet
		}
		auto now = std::chrono::steady_clock::now();
		auto time_since_last_detection = std::chrono::duration_cast<std::chrono::seconds>(now - *last_detection_time_).count();
		return time_since_last_detection <= post_detection_record_time_;
	}

	void setPostDetectionRecordTime(int seconds) {
		std::lock_guard<std::mutex> lock(mutex_);
		post_detection_record_time_ = seconds;
	}

private:
	RecordingManager() : post_detection_record_time_(30) {}

	RecordingManager(const RecordingManager&) = delete;
	RecordingManager& operator=(const RecordingManager&) = delete;

	std::optional<std::chrono::steady_clock::time_point> last_detection_time_;
	int post_detection_record_time_; // in seconds
	std::mutex mutex_;
};