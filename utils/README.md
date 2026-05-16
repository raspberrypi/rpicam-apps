# rpicam-vid Runtime Control

This directory contains tools that enable **live parameter control** of a running
`rpicam-vid` instance — no restart required.

---

## How it works

`rpicam-vid` automatically creates a Unix Domain Socket whose path is derived
from the `--camera` index:

| `--camera` | Socket path |
|---|---|
| `0` (default) | `/tmp/rpicam-vid0.sock` |
| `1` | `/tmp/rpicam-vid1.sock` |

Any client can send plain-text commands (newline-terminated) to change camera
parameters on the fly. The socket is non-blocking and polled once per frame,
so it never interferes with the capture loop.

```
brightness:0.3          ← key:value, newline-terminated
roi:0.25,0.25,0.5,0.5
awb:daylight
```

Send commands from the shell:

```bash
echo "brightness:-0.3"           | nc -U /tmp/rpicam-vid0.sock
echo "awbgains:1.8,1.3"          | nc -U /tmp/rpicam-vid0.sock
printf "ev:1.0\ncontrast:1.2\n"  | nc -U /tmp/rpicam-vid0.sock
```

---

## Multi-camera support

Two `rpicam-vid` instances can run simultaneously on separate cameras, each
with its own independent control socket:

```bash
rpicam-vid --camera 0 -o /dev/null &   # /tmp/rpicam-vid0.sock
rpicam-vid --camera 1 -o /dev/null &   # /tmp/rpicam-vid1.sock
```

Both `rpicam-vid-tui` and `rpicam-vid-gui` can switch between cameras at runtime.

---

## Supported commands

| Command | Value | Range / options |
|---|---|---|
| `brightness:<v>` | float | −1.0 … +1.0 |
| `ev:<v>` | float | −3.0 … +3.0 |
| `contrast:<v>` | float | 0.0 … 3.0 |
| `saturation:<v>` | float | 0.0 … 3.0 |
| `sharpness:<v>` | float | 0.0 … 16.0 |
| `gain:<v>` | float | 1.0 … 16.0 — sets analogue gain mode to manual automatically |
| `awb:<mode>` | string | `auto` `incandescent` `tungsten` `fluorescent` `indoor` `daylight` `cloudy` |
| `awbgains:<r>,<b>` | float,float | manual colour gains — disables auto AWB |
| `roi:<x>,<y>,<w>,<h>` | float×4 | relative 0.0–1.0; full frame = `0,0,1,1` |
| `metering:<mode>` | string | `centre` `spot` `average` `custom` |
| `exposure:<mode>` | string | `normal` `sport` |
| `denoise:<mode>` | string | `auto` `off` `cdn_off` `cdn_fast` `cdn_hq` |
| `hdr:<mode>` | string | `off` `auto` `sensor` `single-exp` |
| `shutter:<µs>` | integer | exposure time in microseconds; `0` = auto |
| `framerate:<fps>` | integer | target frame rate; `0` = auto (sensor maximum) |

> **AWB note:** `awb:<mode>` re-enables auto AWB. `awbgains:<r>,<b>` disables
> auto AWB and applies fixed colour gains. Switching back to any `awb:` mode
> restores automatic white balance.

> **HDR note:** On imx477 (HQ Camera), only `off` and `single-exp` are
> functional at runtime. `auto` and `sensor` both map to single-exposure HDR.

> **Shutter / framerate note:** `shutter:0` restores auto exposure;
> `framerate:0` removes the frame-rate cap. Setting both to non-zero values
> enters fully manual exposure mode. The maximum useful shutter duration is
> 1 / framerate seconds; the GUI and TUI enforce this automatically.

---

## Caps protocol (server → client)

When a client connects, `rpicam-vid` sends a single line describing the active
sensor mode's capabilities:

```
caps:maxfps=40,hasaf=0
```

| Field | Meaning |
|---|---|
| `maxfps` | Maximum frame rate of the active mode (integer) |
| `hasaf` | `1` if the camera supports autofocus, `0` otherwise |

Both `rpicam-vid-gui` and `rpicam-vid-tui` parse this line and clamp the
framerate slider maximum accordingly.

---

## Tools

### `rpicam-vid-tui` — Terminal UI

Interactive terminal UI built with Python 3 stdlib (`curses`) — no additional
dependencies required.

```bash
./rpicam-vid-tui        # camera 0
./rpicam-vid-tui 1      # camera 1
```

**Controls:**

| Key | Action |
|---|---|
| `↑` / `↓` | Move between parameters |
| `←` / `→` | Decrease / increase value (switches camera when Camera row selected) |
| `C` | Cycle to next camera |
| `R` | Reset all parameters to defaults |
| `Q` | Quit |

**Parameters:**

| Row | Type | Notes |
|---|---|---|
| Brightness, EV | slider | |
| Shutter | log-scale slider | 0 = auto; range 100 µs … 1-frame period |
| Framerate | integer slider | 0 = auto; max clamped from sensor caps |
| Contrast, Saturation | slider | |
| Gain, Sharpness | slider | |
| AWB Gain R / B | slider | auto-switches AWB mode to manual |
| Zoom | slider | maps to `roi:` |
| AWB, Metering, Exposure, Denoise, HDR | combo | |

The TUI connects automatically and retries on the next keypress if
`rpicam-vid` is not yet running. On connect, it receives the `caps:` line
and adjusts the framerate slider maximum for the active sensor mode.

**Auto-select cam1:** if `/tmp/rpicam-vid0.sock` does not exist but
`/tmp/rpicam-vid1.sock` does, camera 1 is selected automatically on startup.

Settings are persisted to `/tmp/rpicam-vid{N}.state` (JSON) per camera index
and shared with `rpicam-vid-gui`. Switching cameras restores that camera's
last known values.

---

### `rpicam_vid_gui/` — Qt graphical control panel (`rpicam-vid-gui`)

A Qt Widgets application (Qt6 preferred, Qt5 fallback) with sliders for all continuous parameters and
dropdowns for mode selections. A camera selector switches between camera 0 and
camera 1 at runtime without restarting the tool. Connects automatically to the
socket on startup and reconnects if `rpicam-vid` is restarted.

**Features:**
- Sliders: brightness, EV, shutter (log-scale), framerate (1 fps integer steps),
  contrast, saturation, gain, sharpness, AWB gains R/B, zoom
- AWB mode dropdown + manual R/B gain sliders (auto-switches to manual on move)
- Metering, exposure mode, denoise, HDR dropdowns
- Camera selector (0 / 1) — auto-selects cam 1 if cam 0 socket is absent
- Per-mode fps cap: on connect `rpicam-vid` sends `caps:maxfps=N`;
  the framerate slider is clamped and the current value is adjusted if needed
- Autofocus (AF) controls are implemented but hidden — untested due to
  missing hardware; not production-ready
- Reset button restores all defaults
- Keyboard shortcuts: `R` reset, `Q` quit, `C` switch camera
- Settings persisted to `/tmp/rpicam-vid{N}.state`, shared with `rpicam-vid-tui`
- Reconnects automatically; re-reads state file on each connect

**Build** (requires Qt5 Widgets + Network):

```bash
meson configure build -Denable_control_gui=enabled
ninja -C build
```

---

## State file lifecycle

`rpicam-vid` manages the socket and companion state file as a pair:

| Event | `.sock` | `.state` |
|---|---|---|
| `rpicam-vid` starts | created | deleted (fresh session) |
| `rpicam-vid` stops (SIGINT / SIGTERM) | deleted | deleted |
| Tool connects | — | read by GUI/TUI |
| Tool sends command | — | written atomically |

Each new `rpicam-vid` session therefore starts from hardware defaults regardless
of what a previous session had set. `SIGKILL` cannot be caught; any stale files
are cleaned up when the next session starts.
