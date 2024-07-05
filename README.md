# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform. It is forked from the official repo and we have written our own outputs functionality for the rpicam-raw app.

Official repo status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

Install Raspi Camera
-----

[Here](https://www.raspberrypi.com/documentation/accessories/camera.html#installing-a-raspberry-pi-camera)

```sh
./overlay.sh
```

Reboot now and test afterwards if libcamera-hello works. **Only proceed if it works!** If it doesnt work, make sure `overlay.sh` did work correctly and check `/boot/firmware/config.txt`

Building
------

The following is directly extracted from the [official rpi documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-rpicam-apps-without-building-libcamera). At some point it might be out of date. Just check the documentation for building the apps.

Internal dependency
------

Our app require redis and memcached, so install the dev packages.

```sh
sudo apt install -y libmemcached-dev libhiredis-dev libssl-dev
```

Building rpicam-apps without building libcamera
------

```sh
sudo apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev

sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5

sudo apt install libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
```

Building rpicam-apps
------

```sh
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y meson ninja-build
```

For repeated rebuilds use this script

```sh
./build.sh
```

Memcached installation
------

```sh
sudo apt install memcached
sudo apt install libmemcached-tools
sudo mkdir -p /var/run/memcached/
sudo chmod 775 /var/run/memcached/
sudo chown memcache:memcache /var/run/memcached/
sudo systemctl start memcached
sudo chmod 2750 /var/run/memcached
sudo ldconfig
```

Make sure the memcached configurations are equal to `memcached.conf`. 

The SeamPilot rpicam-app
------

We have developed the following files to suit our usecase of capturing frames and saving them to memcached while streaming an event to redis.

In `output/memcached_output.cpp` we define the procedure of saving the images to memcached and writing an event to redis.

In `output/output.cpp` we define the execution argument mem:// which executes the above routine

This is how we would start the app

Make sure to pass width and height.

```sh
rpicam-raw ---n --framerate 120 --mode 640:400:8 --width 640 --height 400 -o test%05d.raw
rpicam-raw ---n --framerate 120 --mode 640:400:8 --width 640 --height 400 -o mem:// -t 0 --redis localhost:6379 --memcached localhost:11211
rpicam-raw ---n --framerate 120 --mode 1280:800:8 --width 1280 --height 800 -o mem:// -t 0 --redis localhost:6379 --memcached localhost:11211
```

Other options:

- gain
- roi
- exposure

Full list of options
------

Have a look inside `core/options.cpp`

TODO's
------

Search for `TODO` to see what is left to be done.
