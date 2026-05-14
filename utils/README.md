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

> **AWB note:** `awb:<mode>` re-enables auto AWB. `awbgains:<r>,<b>` disables
> auto AWB and applies fixed colour gains. Switching back to any `awb:` mode
> restores automatic white balance.

> **HDR note:** On imx477 (HQ Camera), only `off` and `single-exp` are
> functional at runtime. `auto` and `sensor` both map to single-exposure HDR.

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

All sliders and dropdowns mirror the same parameters as the socket commands.
The TUI connects automatically and retries on the next keypress if
`rpicam-vid` is not yet running.

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
- Sliders: brightness, EV, contrast, saturation, gain, sharpness, zoom
- AWB mode dropdown + manual R/B gain sliders (auto-switches to manual on move)
- Metering, exposure mode, denoise, HDR dropdowns
- Camera selector (0 / 1) — per-camera state, no bleeding between cameras
- Reset button restores all defaults
- Window position saved across restarts
- Keyboard shortcuts: `R` reset, `Q` quit, `C` switch camera
- Settings persisted to `/tmp/rpicam-vid{N}.state`, shared with `rpicam-vid-tui`
- Reconnects automatically; re-reads state file on each connect (shows hardware defaults after camera restart)

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
