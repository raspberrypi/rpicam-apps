/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_video.hpp - video capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "options.hpp"

struct VideoOptions : public Options
{
	VideoOptions() : Options()
	{
		using namespace boost::program_options;
		// Generally we shall use zero or empty values to avoid over-writing the
		// codec's default behaviour.
		// clang-format off
		options_->add_options()
			("bitrate,b", value<std::string>(&v_->bitrate_)->default_value("0bps"),
			 "Set the video bitrate for encoding. If no units are provided, default to bits/second.")
			("profile", value<std::string>(&v_->profile),
			 "Set the encoding profile")
			("level", value<std::string>(&v_->level),
			 "Set the encoding level")
			("intra,g", value<unsigned int>(&v_->intra)->default_value(0),
			 "Set the intra frame period")
			("inline", value<bool>(&v_->inline_headers)->default_value(false)->implicit_value(true),
			 "Force PPS/SPS header with every I frame (h264 only)")
			("codec", value<std::string>(&v_->codec)->default_value("h264"),
			 "Set the codec to use, either h264, libav (if available), mjpeg or yuv420")
			("encoder-libs", value<std::string>(&v_->encoder_libs)->default_value(""),
			 "Set a custom location for the encoder library .so files")
			("save-pts", value<std::string>(&v_->save_pts),
			 "Save a timestamp file with this name")
			("quality,q", value<int>(&v_->quality)->default_value(50),
			 "Set the MJPEG quality parameter (mjpeg only)")
			("listen,l", value<bool>(&v_->listen)->default_value(false)->implicit_value(true),
			 "Listen for an incoming client network connection before sending data to the client")
			("keypress,k", value<bool>(&v_->keypress)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when ENTER pressed")
			("signal,s", value<bool>(&v_->signal)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when signal received")
			("initial,i", value<std::string>(&v_->initial)->default_value("record"),
			 "Use 'pause' to pause the recording at startup, otherwise 'record' (the default)")
			("split", value<bool>(&v_->split)->default_value(false)->implicit_value(true),
			 "Create a new output file every time recording is paused and then resumed")
			("segment", value<uint32_t>(&v_->segment)->default_value(0),
			 "Break the recording into files of approximately this many milliseconds")
			("circular", value<size_t>(&v_->circular)->default_value(0)->implicit_value(4),
			 "Write output to a circular buffer of the given size (in MB) which is saved on exit")
			("frames", value<unsigned int>(&v_->frames)->default_value(0),
			 "Run for the exact number of frames specified. This will override any timeout set.")
			("libav-video-codec", value<std::string>(&v_->libav_video_codec)->default_value("h264_v4l2m2m"),
			 "Sets the libav video codec to use. "
			 "To list available codecs, run  the \"ffmpeg -codecs\" command.")
			("libav-video-codec-opts", value<std::string>(&v_->libav_video_codec_opts),
			 "Sets the libav video codec options to use. "
			 "These override the internal defaults (check 'encoderOptions*()' in 'encoder/libav_encoder.cpp' for the defaults). "
			 "Separate key and value with \"=\" and multiple options with \";\". "
			 "e.g.: \"preset=ultrafast;profile=high;partitions=i8x8,i4x4\". "
			 "To list available options for a given codec, run the \"ffmpeg -h encoder=libx264\" command for libx264.")
			("libav-format", value<std::string>(&v_->libav_format),
			 "Sets the libav encoder output format to use. "
			 "Leave blank to try and deduce this from the filename.\n"
			 "To list available formats, run  the \"ffmpeg -formats\" command.")
			("libav-audio", value<bool>(&v_->libav_audio)->default_value(false)->implicit_value(true),
			 "Records an audio stream together with the video.")
			("audio-codec", value<std::string>(&v_->audio_codec)->default_value("aac"),
			 "Sets the libav audio codec to use.\n"
			 "To list available codecs, run  the \"ffmpeg -codecs\" command.")
			("audio-source", value<std::string>(&v_->audio_source)->default_value("pulse"),
			 "Audio source to record from. Valid options are \"pulse\" and \"alsa\"")
			("audio-device", value<std::string>(&v_->audio_device)->default_value("default"),
			 "Audio device to record from.  To list the available devices,\n"
			 "for pulseaudio, use the following command:\n"
			 "\"pactl list | grep -A2 'Source #' | grep 'Name: '\"\n"
			 "or for alsa, use the following command:\n"
			 "\"arecord -L\"")
			("audio-channels", value<uint32_t>(&v_->audio_channels)->default_value(0),
			 "Number of channels to use for recording audio. Set to 0 to use default value.")
			("audio-bitrate", value<std::string>(&v_->audio_bitrate_)->default_value("32kbps"),
			 "Set the audio bitrate for encoding. If no units are provided, default to bits/second.")
			("audio-samplerate", value<uint32_t>(&v_->audio_samplerate)->default_value(0),
			 "Set the audio sampling rate in Hz for encoding. Set to 0 to use the input sample rate.")
			("av-sync", value<std::string>(&v_->av_sync_)->default_value("0us"),
			 "Add a time offset (in microseconds if no units provided) to the audio stream, relative to the video stream. "
			 "The offset value can be either positive or negative.")
			("low-latency", value<bool>(&v_->low_latency)->default_value(false)->implicit_value(true),
			 "Enables the libav/libx264 low latency presets for video encoding.")
#ifndef DISABLE_RPI_FEATURES
			 ("sync", value<std::string>(&v_->sync_)->default_value("off"),
			  "Whether to synchronise with another camera. Use \"off\", \"server\" or \"client\".")
#endif
		;
		// clang-format on
	}

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (Options::Parse(argc, argv) == false)
			return false;

		return v_->ParseVideo();
	}

	virtual void Print() const override
	{
		Options::Print();
		v_->PrintVideo();
	}
};
