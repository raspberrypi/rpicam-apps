/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * frame_info.hpp - Frame info class for libcamera apps
 */
#include <array>
#include <iomanip>
#include <sstream>
#include <string>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "completed_request.hpp"

struct FrameInfo
{
	FrameInfo(const CompletedRequestPtr &completed_request)
		: exposure_time(0.0), digital_gain(0.0), colour_gains({ { 0.0f, 0.0f } }), focus(0.0), aelock(false),
		  lens_position(-1.0), af_state(0)
	{
		const libcamera::ControlList &ctrls = completed_request->metadata;

		sequence = completed_request->sequence;
		fps = completed_request->framerate;

		auto exp = ctrls.get(libcamera::controls::ExposureTime);
		if (exp)
			exposure_time = *exp;

		auto ag = ctrls.get(libcamera::controls::AnalogueGain);
		if (ag)
			analogue_gain = *ag;

		auto dg = ctrls.get(libcamera::controls::DigitalGain);
		if (dg)
			digital_gain = *dg;

		auto cg = ctrls.get(libcamera::controls::ColourGains);
		if (cg)
		{
			colour_gains[0] = (*cg)[0], colour_gains[1] = (*cg)[1];
		}

		auto fom = ctrls.get(libcamera::controls::FocusFoM);
		if (fom)
			focus = *fom;

		auto ae = ctrls.get(libcamera::controls::AeLocked);
		if (ae)
			aelock = *ae;

		auto lp = ctrls.get(libcamera::controls::LensPosition);
		if (lp)
			lens_position = *lp;

		auto afs = ctrls.get(libcamera::controls::AfState);
		if (afs)
			af_state = *afs;
	}

	std::string ToString(const std::string &info_string) const
	{
		std::string parsed(info_string);

		for (auto const &t : tokens)
		{
			std::size_t pos = parsed.find(t);
			if (pos != std::string::npos)
			{
				std::stringstream value;
				value << std::fixed << std::setprecision(2);

				if (t == "%frame")
					value << sequence;
				else if (t == "%fps")
					value << fps;
				else if (t == "%exp")
					value << exposure_time;
				else if (t == "%ag")
					value << analogue_gain;
				else if (t == "%dg")
					value << digital_gain;
				else if (t == "%rg")
					value << colour_gains[0];
				else if (t == "%bg")
					value << colour_gains[1];
				else if (t == "%focus")
					value << focus;
				else if (t == "%aelock")
					value << aelock;
				else if (t == "%lp")
					value << lens_position;
				else if (t == "%afstate")
				{
					switch (af_state)
					{
					case libcamera::controls::AfStateIdle:
						value << "idle";
						break;
					case libcamera::controls::AfStateScanning:
						value << "scanning";
						break;
					case libcamera::controls::AfStateFocused:
						value << "focused";
						break;
					default:
						value << "failed";
					}
				}

				parsed.replace(pos, t.length(), value.str());
			}
		}

		return parsed;
	}

	unsigned int sequence;
	float exposure_time;
	float analogue_gain;
	float digital_gain;
	std::array<float, 2> colour_gains;
	float focus;
	float fps;
	bool aelock;
	float lens_position;
	int af_state;

private:
	// Info text tokens.
	inline static const std::string tokens[] = { "%frame", "%fps", "%exp", "%ag", "%dg",
						     "%rg", "%bg",  "%focus", "%aelock",
						     "%lp", "%afstate" };
};
