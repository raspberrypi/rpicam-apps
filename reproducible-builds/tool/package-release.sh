#!/usr/bin/env bash
# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPRO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$REPRO_DIR/.." && pwd)"
OUT_DIR="${OUT_DIR:-$REPRO_DIR/out}"
DIST_DIR="${DIST_DIR:-$REPRO_DIR/dist}"
PLATFORM_TAG="${PLATFORM_TAG:-linux-arm64}"

tar_bin() {
  if command -v gtar >/dev/null 2>&1; then
    printf '%s\n' gtar
    return
  fi

  if tar --version 2>/dev/null | grep -q 'GNU tar'; then
    printf '%s\n' tar
    return
  fi

  printf '%s\n' ''
}

sha256_cmd() {
  if command -v sha256sum >/dev/null 2>&1; then
    printf '%s\n' sha256sum
    return
  fi

  if command -v shasum >/dev/null 2>&1; then
    printf '%s\n' "shasum -a 256"
    return
  fi

  echo "Missing sha256 tool. Install sha256sum or shasum." >&2
  exit 1
}

sha256_file() {
  local file_path="$1"
  local checksum_tool="$2"

  if [[ "$checksum_tool" == "sha256sum" ]]; then
    sha256sum "$file_path" | awk '{print $1}'
    return
  fi

  shasum -a 256 "$file_path" | awk '{print $1}'
}

tar_flavor() {
  local tar_tool="$1"

  if [[ -z "$tar_tool" ]]; then
    printf '%s\n' none
    return
  fi

  if "$tar_tool" --version 2>/dev/null | grep -q 'GNU tar'; then
    printf '%s\n' gnu
    return
  fi

  printf '%s\n' bsd
}

package_with_gnu_tar() {
  local tar_tool="$1"
  local tar_root="$2"
  local tar_list="$3"
  local archive_path="$4"

  COPYFILE_DISABLE=1 COPY_EXTENDED_ATTRIBUTES_DISABLE=1 "$tar_tool" \
    --no-recursion \
    --sort=name \
    --format=posix \
    --pax-option=exthdr.name=%d/PaxHeaders/%f,delete=atime,delete=ctime \
    --owner=0 \
    --group=0 \
    --numeric-owner \
    -cf - \
    -C "$tar_root" \
    -T "$tar_list" | gzip -n > "$archive_path"
}

package_with_docker_gnu_tar() {
  local tar_root="$1"
  local tar_list="$2"
  local archive_path="$3"

  command -v docker >/dev/null 2>&1 || {
    echo "GNU tar is required for deterministic packaging. Install gnu-tar or make Docker available." >&2
    exit 1
  }

  docker run --rm \
    -v "$tar_root:/work" \
    -w /work \
    debian:bookworm \
    bash -lc '
      set -euo pipefail
      tar \
        --no-recursion \
        --sort=name \
        --format=posix \
        --pax-option=exthdr.name=%d/PaxHeaders/%f,delete=atime,delete=ctime \
        --owner=0 \
        --group=0 \
        --numeric-owner \
        -cf - \
        -C /work \
        -T /work/file-list.txt | gzip -n
    ' > "$archive_path"
}

touch_timestamp() {
  local epoch="$1"

  if date -u -r 0 +%Y%m%d%H%M.%S >/dev/null 2>&1; then
    date -u -r "$epoch" +%Y%m%d%H%M.%S
    return
  fi

  date -u -d "@$epoch" +%Y%m%d%H%M.%S
}

strip_macos_metadata() {
  local target_dir="$1"

  find "$target_dir" -name '._*' -type f -delete

  if command -v xattr >/dev/null 2>&1; then
    xattr -cr "$target_dir" 2>/dev/null || true
  fi
}

project_version="$(
  sed -n "s/^[[:space:]]*version[[:space:]]*:[[:space:]]*'\\([^']*\\)'.*/\\1/p" "$REPO_DIR/meson.build" | head -n 1
)"

vcs_ref="${VCS_REF:-$(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || printf 'unknown')}"
short_ref="${vcs_ref:0:12}"
release_name="${RELEASE_NAME:-rpicam-apps-${project_version}-${short_ref}-${PLATFORM_TAG}}"
source_date_epoch="${SOURCE_DATE_EPOCH:-$(git -C "$REPO_DIR" log -1 --format=%ct 2>/dev/null || printf '0')}"

[[ -d "$OUT_DIR/usr" ]] || {
  echo "Expected built export tree at $OUT_DIR/usr. Run reproducible-builds/tool/run.sh first." >&2
  exit 1
}

tar_tool="$(tar_bin)"
tar_kind="$(tar_flavor "$tar_tool")"
checksum_tool="$(sha256_cmd)"
tmp_root="$(mktemp -d "${TMPDIR:-/tmp}/rpicam-apps-release.XXXXXX")"
staging_dir="$tmp_root/$release_name"
file_list="$tmp_root/file-list.txt"
timestamp="$(touch_timestamp "$source_date_epoch")"

cleanup() {
  rm -rf "$tmp_root"
}
trap cleanup EXIT

mkdir -p "$DIST_DIR" "$staging_dir"
rsync -a --delete \
  --exclude '.DS_Store' \
  --exclude '._*' \
  "$OUT_DIR"/ "$staging_dir"/
strip_macos_metadata "$staging_dir"
find "$staging_dir" -exec touch -h -t "$timestamp" {} +
(
  cd "$tmp_root"
  {
    printf '%s\n' "$release_name"
    find "$release_name" -mindepth 1 -type d -empty -print
    find "$release_name" -type f -print
    find "$release_name" -type l -print
  } | LC_ALL=C sort -u > "$file_list"
)

archive_path="$DIST_DIR/${release_name}.tar.gz"
checksum_path="${archive_path}.sha256"

if [[ "$tar_kind" == "gnu" ]]; then
  package_with_gnu_tar "$tar_tool" "$tmp_root" "$file_list" "$archive_path"
else
  package_with_docker_gnu_tar "$tmp_root" "$file_list" "$archive_path"
fi

checksum_value="$(sha256_file "$archive_path" "$checksum_tool")"
printf '%s  %s\n' "$checksum_value" "$(basename "$archive_path")" > "$checksum_path"

echo "Wrote release archive: $archive_path"
echo "Wrote checksum: $checksum_path"
