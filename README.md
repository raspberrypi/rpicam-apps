# libcamera-apps

This is a small suite of libcamera-based apps that aim to copy the functionality of the existing "raspicam" apps. 

Build
-----
For usage and build instructions, see the official Raspberry Pi documenation pages [here.](https://www.raspberrypi.com/documentation/accessories/camera.html#libcamera-and-libcamera-apps)

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/libcamera-apps/actions/workflows/libcamera-test.yml/badge.svg)](https://github.com/raspberrypi/libcamera-apps/actions/workflows/libcamera-test.yml)


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
