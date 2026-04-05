# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform.

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links are installed to allow users to keep using the old application names, but these will be deprecated soon. Users are encouraged to adopt the new application and library names as soon as possible.

Build
-----
For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

Reproducible Builds
-------------------
Secluso's reproducible build and verification tooling is located in reproducible-builds/. That subtree contains the Docker-based builder, release packager, and verification notes for checking a packaged release against reproducible-builds/out/. See reproducible-builds/README.md.

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

The Secluso-authored files under reproducible-builds/ are separate tooling and carry their own LICENSE, COPYRIGHT, and NOTICE files.

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)
