#!/bin/sh

cd "$MESON_SOURCE_ROOT" || return
git rev-parse HEAD > "$MESON_DIST_ROOT"/version.gen
