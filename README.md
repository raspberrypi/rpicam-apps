# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform.

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links to allow users to keep using the old application names have now been removed.

Build
-----
For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

GUI Application
---------------

`rpicam-gui` provides a Qt-based interface for configuring and controlling libcamera captures. The window presents a live preview alongside controls for adjusting still and video settings such as resolution, framerate, still encodings, video codecs, bitrate and output locations. Capture buttons trigger still image saves or start and stop video recordings using the existing option structures provided by the command-line tools.

To build the GUI you must enable Qt support when configuring the project:

```
meson setup build -Denable_qt=true
ninja -C build
```

### Raspberry Pi OS (Pi 4B) notes

1. Install the Qt development dependencies:

   ```
   sudo apt update
   sudo apt install qtbase5-dev qtdeclarative5-dev
   ```

2. Configure and build the applications with Qt support as shown above. The build will produce the `rpicam-gui` executable alongside the existing command-line tools.
3. Launch the GUI with:

   ```
   ./build/apps/rpicam-gui
   ```

   Use the preview pane to frame the scene, adjust camera parameters from the settings panel, and press **Capture Photo** or **Start Video** to create still images or video clips.

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)
