#!/usr/bin/env bash
# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPRO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$REPRO_DIR/.." && pwd)"
OUT_DIR="$REPRO_DIR/out"
BAKE_FILE="$SCRIPT_DIR/docker-bake.hcl"
PLATFORM="${PLATFORM:-linux/arm64}"
EXPORT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/rpicam-apps-out.XXXXXX")"
VCS_REF="${VCS_REF:-$(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || true)}"

cleanup() {
  rm -rf "$EXPORT_DIR"
}
trap cleanup EXIT

mkdir -p "$OUT_DIR"

# Export to a temp dir first so removed files do not linger in the checked-in out/ tree.
cd "$REPO_DIR"

# Keep the imperative shell thin and let buildx bake own the build configuration.
docker_env=(
  "PLATFORM=$PLATFORM"
  "OUTPUT_DIR=$EXPORT_DIR"
)

if [[ -n "$VCS_REF" ]]; then
  docker_env+=("VCS_REF=$VCS_REF")
fi

env "${docker_env[@]}" docker buildx bake \
  --allow "fs.write=$EXPORT_DIR" \
  -f "$BAKE_FILE" \
  export

rsync -a --delete "$EXPORT_DIR"/ "$OUT_DIR"/
