/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Kletternaut
 *
 * control_socket.hpp - Unix Domain Socket for runtime camera control in rpicam-vid.
 *
 * Listens on /tmp/rpicam-vid.sock for newline-terminated commands of the form
 *   key:value
 * and translates them into libcamera ControlList entries that can be passed to
 * RPiCamApp::SetControls().
 *
 * Supported commands:
 *   brightness:<float>       -1.0 … 1.0
 *   contrast:<float>          0.0 … 32.0
 *   saturation:<float>        0.0 … 32.0
 *   ev:<float>               exposure compensation in stops
 *   shutter:<int>            exposure time in microseconds (>0); shutter:0 restores AE
 *   framerate:<float>        target frame rate in fps (>0); framerate:0 restores AE-managed rate
 *   awb:<mode>               auto|incandescent|tungsten|fluorescent|indoor|daylight|cloudy|custom
 *   awbgains:<r>,<b>         manual colour gains, e.g. "awbgains:1.5,1.2"
 *   roi:<x>,<y>,<w>,<h>     relative values 0.0-1.0 relative to sensor area
 *   hdr:<mode>               off|auto|sensor|single-exp
 *
 *   gain:<float>              analogue gain (> 0.0, e.g. 1.0)
 *   metering:<mode>          centre|spot|average|custom
 *   exposure:<mode>          normal|sport
 *   sharpness:<float>        0.0 … 15.99 (1.0 = normal)
 *   denoise:<mode>           auto|off|cdn_off|cdn_fast|cdn_hq
 *   af:<mode>                manual|auto|continuous|trigger|cancel
 *   lens:<float>             manual lens position (0.0 = infinity); implies af:manual
 *
 * Usage example (shell):
 *   echo "brightness:0.3" | nc -U /tmp/rpicam-vid.sock
 *   echo -e "contrast:1.5\nev:-1.0" | nc -U /tmp/rpicam-vid.sock
 */

#pragma once

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "core/logging.hpp"

static constexpr const char *CONTROL_SOCKET_DEFAULT_PATH = "/tmp/rpicam-vid.sock";

class ControlSocket
{
public:
	explicit ControlSocket(const std::string &path = CONTROL_SOCKET_DEFAULT_PATH) : socket_path_(path)
	{
		server_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
		if (server_fd_ < 0)
		{
			LOG_ERROR("ControlSocket: socket() failed: " << strerror(errno));
			return;
		}

		// Remove a stale socket file from a previous run.
		::unlink(socket_path_.c_str());

		struct sockaddr_un addr {};
		addr.sun_family = AF_UNIX;
		strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

		if (::bind(server_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
		{
			LOG_ERROR("ControlSocket: bind() failed: " << strerror(errno));
			::close(server_fd_);
			server_fd_ = -1;
			return;
		}

		if (::listen(server_fd_, 4) < 0)
		{
			LOG_ERROR("ControlSocket: listen() failed: " << strerror(errno));
			::close(server_fd_);
			server_fd_ = -1;
			return;
		}

		LOG(1, "ControlSocket: listening on " << socket_path_);

		// Remove any leftover state file from a previous GUI/TUI session so
		// every new rpicam-vid run starts with hardware defaults.
		::unlink(statePath().c_str());
	}

	~ControlSocket()
	{
		if (client_fd_ >= 0)
			::close(client_fd_);
		if (server_fd_ >= 0)
		{
			::close(server_fd_);
			::unlink(socket_path_.c_str());
			// Remove the companion state file so the GUI/TUI resets to
			// defaults when the next rpicam-vid session starts.
			::unlink(statePath().c_str());
		}
	}

	// Non-copyable, non-movable.
	ControlSocket(const ControlSocket &) = delete;
	ControlSocket &operator=(const ControlSocket &) = delete;

	bool IsValid() const
	{
		return server_fd_ >= 0;
	}

	// Accept a pending connection (non-blocking). A new connection replaces
	// an existing one. Call once per iteration of the main loop.
	void AcceptConnections()
	{
		if (server_fd_ < 0)
			return;

		int fd = ::accept4(server_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
		if (fd >= 0)
		{
			if (client_fd_ >= 0)
				::close(client_fd_);
			client_fd_ = fd;
			recv_buf_.clear();
			newClientConnected_ = true;
			LOG(1, "ControlSocket: client connected");
		}
	}

	// Returns true once after a new client connected; cleared by ClearNewClient().
	bool HasNewClient() const
	{
		return newClientConnected_;
	}
	void ClearNewClient()
	{
		newClientConnected_ = false;
	}

	// Send a raw message to the connected client (non-blocking, fire-and-forget).
	void SendToClient(const std::string &msg)
	{
		if (client_fd_ < 0)
			return;
		::send(client_fd_, msg.c_str(), msg.size(), MSG_NOSIGNAL);
	}

	// Drain the receive buffer of any pending data and return a ControlList
	// built from all complete commands. Returns an empty ControlList when
	// nothing has arrived. sensor_area is required for ROI → pixel conversion.
	// is_pisp selects ScalerCrops (PISP/Pi5) vs. ScalerCrop (VC4/Pi4).
	libcamera::ControlList ReadControls(const libcamera::Rectangle &sensor_area, bool is_pisp = false)
	{
		libcamera::ControlList cl(libcamera::controls::controls);

		if (client_fd_ < 0)
			return cl;

		// Read all available data without blocking.
		char buf[256];
		ssize_t n;
		while ((n = ::recv(client_fd_, buf, sizeof(buf) - 1, 0)) > 0)
		{
			buf[n] = '\0';
			recv_buf_ += buf;
		}

		bool peer_closed = (n == 0);
		if (peer_closed)
		{
			// Orderly shutdown by the peer — process remaining buffer first.
			::close(client_fd_);
			client_fd_ = -1;
		}
		// n < 0 with EAGAIN/EWOULDBLOCK is expected on a non-blocking socket.

		// Process every complete (newline-terminated) command.
		std::string::size_type pos;
		while ((pos = recv_buf_.find('\n')) != std::string::npos)
		{
			std::string line = recv_buf_.substr(0, pos);
			recv_buf_.erase(0, pos + 1);

			// Strip optional carriage return (e.g. from Windows clients).
			if (!line.empty() && line.back() == '\r')
				line.pop_back();

			if (!line.empty())
				parseCommand(line, sensor_area, is_pisp, cl);
		}

		return cl;
	}

private:
	int server_fd_ = -1;
	int client_fd_ = -1;
	bool newClientConnected_ = false;
	std::string socket_path_;

	// Derive the state-file path from the socket path by replacing the
	// ".sock" suffix with ".state" (e.g. /tmp/rpicam-vid0.state).
	std::string statePath() const
	{
		const std::string suffix = ".sock";
		if (socket_path_.size() > suffix.size() &&
			socket_path_.compare(socket_path_.size() - suffix.size(), suffix.size(), suffix) == 0)
		{
			return socket_path_.substr(0, socket_path_.size() - suffix.size()) + ".state";
		}
		return socket_path_ + ".state";
	}
	std::string recv_buf_;

	static void parseCommand(const std::string &line, const libcamera::Rectangle &sensor_area, bool is_pisp,
							 libcamera::ControlList &cl)
	{
		const auto colon = line.find(':');
		if (colon == std::string::npos)
		{
			LOG_ERROR("ControlSocket: malformed command (missing ':'): " << line);
			return;
		}

		const std::string key = line.substr(0, colon);
		const std::string val = line.substr(colon + 1);

		// Locale-independent float parser: force '.' as the decimal separator.
		const std::locale classic = std::locale::classic();
		auto parseFloat = [&](const std::string &s) -> float
		{
			std::istringstream ss(s);
			ss.imbue(classic);
			float v;
			ss >> v;
			if (ss.fail())
				throw std::invalid_argument("invalid float: " + s);
			return v;
		};
		// Parse a comma-separated list of floats into a vector (locale-independent).
		auto parseFloatList = [&](const std::string &s, std::vector<float> &out) -> bool
		{
			std::istringstream ss(s);
			ss.imbue(classic);
			float v;
			while (ss >> v)
			{
				out.push_back(v);
				char comma;
				ss >> comma; // consume the comma separator (or EOF)
			}
			return !out.empty();
		};

		try
		{
			if (key == "brightness")
			{
				const float v = std::clamp(parseFloat(val), -1.0f, 1.0f);
				cl.set(libcamera::controls::Brightness, v);
				LOG(1, "ControlSocket: Brightness=" << v);
			}
			else if (key == "contrast")
			{
				const float v = std::clamp(parseFloat(val), 0.0f, 32.0f);
				cl.set(libcamera::controls::Contrast, v);
				LOG(1, "ControlSocket: Contrast=" << v);
			}
			else if (key == "saturation")
			{
				const float v = std::clamp(parseFloat(val), 0.0f, 32.0f);
				cl.set(libcamera::controls::Saturation, v);
				LOG(1, "ControlSocket: Saturation=" << v);
			}
			else if (key == "ev")
			{
				const float v = parseFloat(val);
				cl.set(libcamera::controls::AeEnable, true);
				cl.set(libcamera::controls::ExposureValue, v);
				LOG(1, "ControlSocket: ExposureValue=" << v);
			}
			else if (key == "shutter")
			{
				const int us = std::stoi(val);
				if (us == 0)
				{
					cl.set(libcamera::controls::AeEnable, true);
					LOG(1, "ControlSocket: ExposureTime=auto");
				}
				else if (us > 0)
				{
					cl.set(libcamera::controls::AeEnable, false);
					cl.set(libcamera::controls::ExposureTime, us);
					LOG(1, "ControlSocket: ExposureTime=" << us << " us");
				}
				else
					LOG_ERROR("ControlSocket: shutter must be >= 0");
			}
			else if (key == "framerate")
			{
				const float fps = parseFloat(val);
				if (fps == 0.0f)
				{
					// Restore AE-managed framerate with a wide open range
					cl.set(libcamera::controls::FrameDurationLimits,
						   libcamera::Span<const int64_t, 2>({ 100LL, 1000000LL }));
					LOG(1, "ControlSocket: FrameDurationLimits=auto");
				}
				else if (fps > 0.0f)
				{
					const int64_t dur = static_cast<int64_t>(1e6f / fps);
					cl.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ dur, dur }));
					LOG(1, "ControlSocket: FrameDurationLimits=" << dur << " us (" << fps << " fps)");
				}
				else
					LOG_ERROR("ControlSocket: framerate must be >= 0");
			}
			else if (key == "awb")
			{
				// clang-format off
				static const std::map<std::string, int32_t> awb_table =
				{
					{ "auto",         libcamera::controls::AwbAuto        },
					{ "normal",       libcamera::controls::AwbAuto        },
					{ "incandescent", libcamera::controls::AwbIncandescent },
					{ "tungsten",     libcamera::controls::AwbTungsten     },
					{ "fluorescent",  libcamera::controls::AwbFluorescent  },
					{ "indoor",       libcamera::controls::AwbIndoor       },
					{ "daylight",     libcamera::controls::AwbDaylight     },
					{ "cloudy",       libcamera::controls::AwbCloudy       },
					{ "custom",       libcamera::controls::AwbCustom       },
				};
				// clang-format on
				const auto it = awb_table.find(val);
				if (it != awb_table.end())
				{
					cl.set(libcamera::controls::AwbEnable, true);
					cl.set(libcamera::controls::AwbMode, it->second);
					LOG(1, "ControlSocket: AwbMode=" << val);
				}
				else
					LOG_ERROR("ControlSocket: unknown AWB mode: " << val);
			}
			else if (key == "awbgains")
			{
				std::vector<float> gains;
				if (parseFloatList(val, gains) && gains.size() == 2 && gains[0] > 0.0f && gains[1] > 0.0f)
				{
					cl.set(libcamera::controls::AwbEnable, false);
					cl.set(libcamera::controls::ColourGains, libcamera::Span<const float, 2>({ gains[0], gains[1] }));
					LOG(1, "ControlSocket: ColourGains=" << gains[0] << "," << gains[1]);
				}
				else
					LOG_ERROR("ControlSocket: invalid awbgains (expect r,b with r>0 b>0): " << val);
			}
			else if (key == "roi")
			{
				std::vector<float> rv;
				if (parseFloatList(val, rv) && rv.size() == 4)
				{
					float x = std::clamp(rv[0], 0.0f, 1.0f);
					float y = std::clamp(rv[1], 0.0f, 1.0f);
					float w = std::clamp(rv[2], 0.0f, 1.0f - x);
					float h = std::clamp(rv[3], 0.0f, 1.0f - y);

					libcamera::Rectangle crop(sensor_area.x + static_cast<int>(x * sensor_area.width),
											  sensor_area.y + static_cast<int>(y * sensor_area.height),
											  static_cast<unsigned int>(w * sensor_area.width),
											  static_cast<unsigned int>(h * sensor_area.height));

					// ScalerCrop  → VC4 (Pi 1–4); ScalerCrops → PISP (Pi 5)
					if (is_pisp)
					{
						const std::vector<libcamera::Rectangle> crops = { crop };
						cl.set(libcamera::controls::rpi::ScalerCrops,
							   libcamera::Span<const libcamera::Rectangle>(crops.data(), crops.size()));
					}
					else
					{
						cl.set(libcamera::controls::ScalerCrop, crop);
					}
					LOG(1, "ControlSocket: ScalerCrop(s)=" << crop.toString());
				}
				else
					LOG_ERROR("ControlSocket: invalid roi (expect x,y,w,h 0.0-1.0): " << val);
			}
			else if (key == "hdr")
			{
				// clang-format off
				static const std::map<std::string, int32_t> hdr_table =
				{
					{ "off",        libcamera::controls::HdrModeOff            },
					{ "auto",       libcamera::controls::HdrModeSingleExposure },
					{ "sensor",     libcamera::controls::HdrModeSingleExposure },
					{ "single-exp", libcamera::controls::HdrModeSingleExposure },
				};
				// clang-format on
				const auto it = hdr_table.find(val);
				if (it != hdr_table.end())
				{
					cl.set(libcamera::controls::HdrMode, it->second);
					LOG(1, "ControlSocket: HdrMode=" << val);
				}
				else
					LOG_ERROR("ControlSocket: unknown HDR mode: " << val);
			}
			else if (key == "gain")
			{
				const float v = parseFloat(val);
				if (v == 0.0f)
				{
					cl.set(libcamera::controls::AnalogueGainMode, libcamera::controls::AnalogueGainModeAuto);
					LOG(1, "ControlSocket: AnalogueGain=auto");
				}
				else if (v > 0.0f)
				{
					cl.set(libcamera::controls::AnalogueGainMode, libcamera::controls::AnalogueGainModeManual);
					cl.set(libcamera::controls::AnalogueGain, v);
					LOG(1, "ControlSocket: AnalogueGain=" << v);
				}
				else
					LOG_ERROR("ControlSocket: gain must be >= 0");
			}
			else if (key == "metering")
			{
				// clang-format off
				static const std::map<std::string, int32_t> metering_table =
				{
					{ "centre",  libcamera::controls::MeteringCentreWeighted },
					{ "center",  libcamera::controls::MeteringCentreWeighted },
					{ "spot",    libcamera::controls::MeteringSpot           },
					{ "average", libcamera::controls::MeteringMatrix         },
					{ "matrix",  libcamera::controls::MeteringMatrix         },
					{ "custom",  libcamera::controls::MeteringCustom         },
				};
				// clang-format on
				const auto it = metering_table.find(val);
				if (it != metering_table.end())
				{
					cl.set(libcamera::controls::AeMeteringMode, it->second);
					LOG(1, "ControlSocket: AeMeteringMode=" << val);
				}
				else
					LOG_ERROR("ControlSocket: unknown metering mode: " << val);
			}
			else if (key == "exposure")
			{
				// clang-format off
				static const std::map<std::string, int32_t> exposure_table =
				{
					{ "normal", libcamera::controls::ExposureNormal },
					{ "sport",  libcamera::controls::ExposureShort  },
					{ "short",  libcamera::controls::ExposureShort  },
					{ "long",   libcamera::controls::ExposureLong   },
					{ "custom", libcamera::controls::ExposureCustom },
				};
				// clang-format on
				const auto it = exposure_table.find(val);
				if (it != exposure_table.end())
				{
					cl.set(libcamera::controls::AeExposureMode, it->second);
					LOG(1, "ControlSocket: AeExposureMode=" << val);
				}
				else
					LOG_ERROR("ControlSocket: unknown exposure mode: " << val);
			}
			else if (key == "sharpness")
			{
				const float v = std::clamp(parseFloat(val), 0.0f, 15.99f);
				cl.set(libcamera::controls::Sharpness, v);
				LOG(1, "ControlSocket: Sharpness=" << v);
			}
			else if (key == "denoise")
			{
				// clang-format off
				static const std::map<std::string, int32_t> denoise_table =
				{
					{ "auto",     libcamera::controls::draft::NoiseReductionModeMinimal     },
					{ "off",      libcamera::controls::draft::NoiseReductionModeOff         },
					{ "cdn_off",  libcamera::controls::draft::NoiseReductionModeMinimal     },
					{ "cdn_fast", libcamera::controls::draft::NoiseReductionModeFast        },
					{ "cdn_hq",   libcamera::controls::draft::NoiseReductionModeHighQuality },
				};
				// clang-format on
				const auto it = denoise_table.find(val);
				if (it != denoise_table.end())
				{
					cl.set(libcamera::controls::draft::NoiseReductionMode, it->second);
					LOG(1, "ControlSocket: NoiseReductionMode=" << val);
				}
				else
					LOG_ERROR("ControlSocket: unknown denoise mode: " << val);
			}
			else if (key == "af")
			{
				if (val == "trigger")
				{
					cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
					LOG(1, "ControlSocket: AfTrigger=start");
				}
				else if (val == "cancel")
				{
					cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerCancel);
					LOG(1, "ControlSocket: AfTrigger=cancel");
				}
				else
				{
					// clang-format off
					static const std::map<std::string, int32_t> af_table =
					{
						{ "manual",     libcamera::controls::AfModeManual     },
						{ "auto",       libcamera::controls::AfModeAuto       },
						{ "continuous", libcamera::controls::AfModeContinuous },
					};
					// clang-format on
					const auto it = af_table.find(val);
					if (it != af_table.end())
					{
						cl.set(libcamera::controls::AfMode, it->second);
						LOG(1, "ControlSocket: AfMode=" << val);
					}
					else
						LOG_ERROR("ControlSocket: unknown af mode: " << val);
				}
			}
			else if (key == "lens")
			{
				const float v = parseFloat(val);
				if (v >= 0.0f)
				{
					cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
					cl.set(libcamera::controls::LensPosition, v);
					LOG(1, "ControlSocket: LensPosition=" << v);
				}
				else
					LOG_ERROR("ControlSocket: lens position must be >= 0");
			}
			else
			{
				LOG_ERROR("ControlSocket: unknown command key: " << key);
			}
		}
		catch (const std::exception &e)
		{
			LOG_ERROR("ControlSocket: error parsing '" << line << "': " << e.what());
		}
	}
};
