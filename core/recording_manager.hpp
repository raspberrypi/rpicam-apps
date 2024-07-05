#pragma once

#include <mutex>
#include <chrono>

class RecordingManager {
public:
	static RecordingManager& getInstance() {
		static RecordingManager instance;
		return instance;
	}

	void objectDetected() {
		std::lock_guard<std::mutex> lock(mutex_);
		last_detection_time_ = std::chrono::steady_clock::now();
	}

	bool shouldRecord() {
		std::lock_guard<std::mutex> lock(mutex_);
		auto now = std::chrono::steady_clock::now();
		auto time_since_last_detection = std::chrono::duration_cast<std::chrono::seconds>(now - last_detection_time_).count();
		return time_since_last_detection <= post_detection_record_time_;
	}

	void setPostDetectionRecordTime(int seconds) {
		std::lock_guard<std::mutex> lock(mutex_);
		post_detection_record_time_ = seconds;
	}

private:
	RecordingManager() : last_detection_time_(std::chrono::steady_clock::now()), post_detection_record_time_(30) {}

	RecordingManager(const RecordingManager&) = delete;
	RecordingManager& operator=(const RecordingManager&) = delete;

	std::chrono::steady_clock::time_point last_detection_time_;
	int post_detection_record_time_; // in seconds
	std::mutex mutex_;
};
