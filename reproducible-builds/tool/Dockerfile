# syntax=docker/dockerfile:1.7
# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later
FROM debian:bookworm AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG LIBCAMERA_VERSION=0.4.0+rpt20250213-1
ARG LIBCAMERA_ARCH=arm64
ARG RPI_DIST=bookworm
ARG SOURCE_DATE_EPOCH=0
ARG VCS_REF=
ARG LIBCAMERA_POOL_URL=https://mirror.fsmg.org.nz/pub/raspberrypi/debian/pool/main/libc/libcamera

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      git \
      gnupg \
      libboost-program-options-dev \
      libdrm-dev \
      libepoxy-dev \
      libexif-dev \
      libjpeg-dev \
      libpng-dev \
      libtiff5-dev \
      meson \
      ninja-build \
      pkg-config && \
    rm -rf /var/lib/apt/lists/*

# Pull Raspberry Pi's libcamera/libpisp packages from the Pi archive while blocking newer 0.5 packages.
RUN install -d -m 0755 /usr/share/keyrings && \
    curl -fsSL "https://archive.raspberrypi.org/debian/raspberrypi.gpg.key" | \
      gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg && \
    printf '%s\n' \
      "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.org/debian/ ${RPI_DIST} main" \
      > /etc/apt/sources.list.d/raspberrypi.list && \
    printf '%s\n' \
      'Package: libcamera0.4 libcamera-ipa libcamera-dev libcamera-tools python3-libcamera' \
      'Pin: version 0.4.*' \
      'Pin-Priority: 1001' \
      '' \
      'Package: libcamera0.5 libcamera-ipa' \
      'Pin: version 0.5.*' \
      'Pin-Priority: -10' \
      > /etc/apt/preferences.d/rpicam-libcamera

RUN set -eux; \
    apt-get update; \
    apt-get remove -y 'libcamera0.5*' 'libcamera-ipa' 'libcamera-tools' 'libcamera-dev' 'python3-libcamera*' || true; \
    apt-get autoremove -y || true; \
    cd /tmp; \
    debs=( \
      "libcamera0.4_${LIBCAMERA_VERSION}_${LIBCAMERA_ARCH}.deb" \
      "libcamera-ipa_${LIBCAMERA_VERSION}_${LIBCAMERA_ARCH}.deb" \
      "libcamera-dev_${LIBCAMERA_VERSION}_${LIBCAMERA_ARCH}.deb" \
    ); \
    for deb in "${debs[@]}"; do \
      curl -fsSLo "$deb" "${LIBCAMERA_POOL_URL}/$deb"; \
    done; \
    dpkg -i "${debs[@]}" || true; \
    apt-get update; \
    apt-get install -y --no-install-recommends -f; \
    dpkg -i "${debs[@]}"; \
    apt-mark hold libcamera0.4 libcamera-ipa libcamera-dev || true; \
    rm -f "${debs[@]}"; \
    rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

RUN set -eux; \
    export SOURCE_DATE_EPOCH; \
    if [[ -n "${VCS_REF}" ]]; then \
      printf '%s\n' "${VCS_REF}" > /src/version.gen; \
    fi; \
    meson setup /build \
      --prefix=/usr \
      --buildtype=release \
      -Denable_libav=disabled \
      -Denable_drm=enabled \
      -Denable_egl=disabled \
      -Denable_qt=disabled \
      -Denable_opencv=disabled \
      -Denable_tflite=disabled \
      -Denable_hailo=disabled && \
    meson compile -C /build -j 1

RUN set -eux; \
    DESTDIR=/staging meson install -C /build --no-rebuild; \
    rm -rf /staging/usr/include; \
    install -d /staging/manifest; \
    find /staging/usr/bin -maxdepth 1 \( -type f -o -type l \) | sort > /staging/manifest/installed-binaries.txt; \
    find /staging/usr/lib -maxdepth 3 \( -type f -o -type l \) | sort > /staging/manifest/installed-libs.txt

FROM scratch AS export
COPY --from=build /staging/ /
