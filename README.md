# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform.

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links are installed to allow users to keep using the old application names, but these will be deprecated soon. Users are encouraged to adopt the new application and library names as soon as possible.

Build
-----
For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

License
-------

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

## Pre

```
sudo apt install libmemcached-dev
```

## Building from source

```
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
mkdir build
cd build
cmake .. -DENABLE_DRM=1 -DENABLE_X11=1 -DENABLE_QT=1 -DENABLE_OPENCV=0 -DENABLE_TFLITE=0
make -j4  # use -j1 on Pi 3 or earlier devices
sudo make install
sudo ldconfig # this is only necessary on the first build
```


## Memcached

This works currently only with memcached unix sockets installations and redis


```
./install.sh
```

Afterwards you can use the -o flag with mem:// to write raw image bytes to memcached and stream the keys in redis


## Memcached output flag

```
libcamera-raw --width 640 --height 400 ---n -v --framerate 120 -o mem://
```
