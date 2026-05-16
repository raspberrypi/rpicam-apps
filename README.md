# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform.

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links to allow users to keep using the old application names have now been removed.

Runtime Control (feature/runtime-control-socket)
-------------------------------------------------

`rpicam-vid` listens on a Unix domain socket and accepts plain-text commands at runtime — no restart required. The socket path is derived from the `--camera` index:

| `--camera` | Socket |
|---|---|
| `0` (default) | `/tmp/rpicam-vid0.sock` |
| `1` | `/tmp/rpicam-vid1.sock` |

### Supported commands

| Command | Example | Effect |
|---|---|---|
| `shutter:<µs>` | `shutter:10000` | Fixed shutter 0 (auto) … sensor max µs |
| `framerate:<fps>` | `framerate:30` | Fixed framerate 0 (auto) … sensor max fps |
| `brightness:<val>` | `brightness:-0.2` | Brightness −1.0 … +1.0 |
| `ev:<val>` | `ev:1.5` | EV compensation −3.0 … +3.0 |
| `contrast:<val>` | `contrast:1.2` | Contrast 0.0 … 3.0 |
| `saturation:<val>` | `saturation:0.8` | Saturation 0.0 … 3.0 |
| `sharpness:<val>` | `sharpness:1.5` | Sharpness 0.0 … 16.0 |
| `gain:<val>` | `gain:4.0` | Analogue gain 1.0 … 16.0 |
| `awb:<mode>` | `awb:daylight` | AWB mode: auto \| incandescent \| tungsten \| fluorescent \| indoor \| daylight \| cloudy |
| `awbgains:<r>,<b>` | `awbgains:1.8,1.3` | Manual colour gains (disables auto AWB) |
| `roi:<x>,<y>,<w>,<h>` | `roi:0.25,0.25,0.5,0.5` | Digital zoom / ROI, relative 0.0–1.0 |
| `metering:<mode>` | `metering:spot` | Metering: centre \| spot \| average \| custom |
| `exposure:<mode>` | `exposure:sport` | Exposure mode: normal \| sport |
| `denoise:<mode>` | `denoise:cdn_fast` | Denoise: auto \| off \| cdn_off \| cdn_fast \| cdn_hq |
| `hdr:<mode>` | `hdr:single-exp` | HDR: off \| auto \| sensor \| single-exp |

Send commands via `echo` or `nc`:

```sh
echo "brightness:-0.3"        | nc -U /tmp/rpicam-vid0.sock
echo "awb:daylight"           | nc -U /tmp/rpicam-vid0.sock
echo "roi:0.25,0.25,0.5,0.5" | nc -U /tmp/rpicam-vid0.sock
```

### Qt GUI (`rpicam-vid-gui`)

A graphical control panel built with Qt Widgets. Connects to the running `rpicam-vid` socket and provides:

- **Sliders** for brightness, EV, contrast, saturation, sharpness, gain, zoom, AWB gains, shutter (logarithmic scale), and framerate (1 fps integer steps)
- **Dropdowns** for AWB, metering, exposure, denoise, HDR modes
- **Autofocus (AF) controls** are implemented in code but hidden in the UI — untested due to missing hardware (no AF-capable lens available); not production-ready
- **Camera selector** (Cam 0 / Cam 1) — auto-selects Cam 1 if Cam 0 is not detected
- **Per-mode fps cap**: on connect, `rpicam-vid` reports the active sensor mode's maximum fps; the framerate slider is clamped accordingly
- **Keyboard shortcuts**: `R` reset all, `Q` quit, `C` switch camera

Build via Meson:

```sh
meson configure build -Denable_control_gui=enabled
ninja -C build
```

Requires Qt6 or Qt5 Widgets + Network. Connects automatically to the socket on startup and reconnects if `rpicam-vid` is restarted.

### TUI (`rpicam-vid-tui`)

A terminal-based ASCII slider interface, requires only Python 3 (stdlib `curses`):

```sh
./utils/rpicam-vid-tui        # camera 0
./utils/rpicam-vid-tui 1      # camera 1
```

Navigate with `↑`/`↓`, adjust values with `←`/`→`, `C` to switch camera, `R` to reset all, `Q` to quit.

### Per-camera state persistence

Both tools maintain independent settings per camera index and persist them to `/tmp/rpicam-vid{N}.state`. Switching cameras restores that camera's last values. When `rpicam-vid` starts or ends a session it removes any stale state file, so both tools reset to hardware defaults on reconnect.

Build
-----
For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

For Developers
--------------

This project uses [pre-commit](https://pre-commit.com/) to run formatting and linting checks on each commit. To install:

```sh
pip install pre-commit
pre-commit install
pre-commit install --hook-type commit-msg
```

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)
