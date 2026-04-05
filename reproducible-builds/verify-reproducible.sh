#!/usr/bin/env bash
# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set -euo pipefail

usage() {
  cat <<EOF
Usage:
  $0 [--release-dir /path/to/release] [--archive /path/to/archive.tar.gz] [--skip-build] [--skip-signatures]

Default release dir:
  reproducible-builds/release/

Expected release assets:
  <archive>.tar.gz
  <archive>.tar.gz.sha256
  <archive>.tar.gz.sha256.<label>.asc (optional but recommended)
EOF
  exit 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="${RELEASE_DIR:-$SCRIPT_DIR/release}"
ARCHIVE_PATH=""
SKIP_BUILD=0
SKIP_SIGNATURES=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --release-dir) RELEASE_DIR="$2"; shift 2 ;;
    --archive) ARCHIVE_PATH="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --skip-signatures) SKIP_SIGNATURES=1; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown arg: $1" >&2; usage ;;
  esac
done

hash_file() {
  local file_path="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$file_path" | awk '{print $1}'
  else
    shasum -a 256 "$file_path" | awk '{print $1}'
  fi
}

resolve_archive_path() {
  local candidate="$1"

  if [[ -n "$candidate" ]]; then
    printf '%s\n' "$candidate"
    return
  fi

  [[ -d "$RELEASE_DIR" ]] || {
    echo "Release directory not found: $RELEASE_DIR" >&2
    exit 1
  }

  local archives=()
  while IFS= read -r path; do
    archives+=("$path")
  done < <(find "$RELEASE_DIR" -maxdepth 1 -type f -name '*.tar.gz' | sort)

  if [[ "${#archives[@]}" -eq 0 ]]; then
    echo "No .tar.gz release archive found in $RELEASE_DIR" >&2
    exit 1
  fi

  if [[ "${#archives[@]}" -ne 1 ]]; then
    echo "Found multiple .tar.gz archives in $RELEASE_DIR; pass --archive explicitly." >&2
    printf '  %s\n' "${archives[@]}" >&2
    exit 1
  fi

  printf '%s\n' "${archives[0]}"
}

verify_checksum() {
  local archive_path="$1"
  local checksum_path="$2"

  [[ -f "$checksum_path" ]] || {
    echo "Missing checksum file: $checksum_path" >&2
    exit 1
  }

  local expected_line expected_hash expected_name actual_hash actual_name
  expected_line="$(LC_ALL=C head -n 1 "$checksum_path" | tr -d '\r')"
  expected_hash="$(printf '%s\n' "$expected_line" | awk '{print $1}')"
  expected_name="$(printf '%s\n' "$expected_line" | awk '{print $2}' | sed 's/^\*//')"
  actual_hash="$(hash_file "$archive_path")"
  actual_name="$(basename "$archive_path")"

  [[ -n "$expected_hash" && -n "$expected_name" ]] || {
    echo "Malformed checksum file: $checksum_path" >&2
    exit 1
  }

  [[ "$expected_name" == "$actual_name" ]] || {
    echo "Checksum file does not refer to $actual_name: $expected_name" >&2
    exit 1
  }

  [[ "$expected_hash" == "$actual_hash" ]] || {
    echo "Archive checksum mismatch for $actual_name" >&2
    echo "Expected: $expected_hash" >&2
    echo "Actual:   $actual_hash" >&2
    exit 1
  }
}

verify_signatures() {
  local checksum_path="$1"

  if [[ "$SKIP_SIGNATURES" -eq 1 ]]; then
    echo "Skipping GPG signature verification."
    return
  fi

  local sig_files=()
  while IFS= read -r path; do
    sig_files+=("$path")
  done < <(find "$(dirname "$checksum_path")" -maxdepth 1 -type f -name "$(basename "$checksum_path").*.asc" | sort)

  if [[ "${#sig_files[@]}" -eq 0 ]]; then
    echo "No checksum signatures found next to $(basename "$checksum_path"); skipping GPG verification."
    return
  fi

  local sig_file
  for sig_file in "${sig_files[@]}"; do
    echo "Verifying signature: $(basename "$sig_file")"
    gpg --verify "$sig_file" "$checksum_path"
  done
}

ARCHIVE_PATH="$(resolve_archive_path "$ARCHIVE_PATH")"
CHECKSUM_PATH="${ARCHIVE_PATH}.sha256"
TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/rpicam-apps-verify.XXXXXX")"
DIFF_PATH="$TMP_DIR/release-vs-out.diff"

cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

echo "Verifying checksum for $(basename "$ARCHIVE_PATH")"
verify_checksum "$ARCHIVE_PATH" "$CHECKSUM_PATH"
verify_signatures "$CHECKSUM_PATH"

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  echo "Rebuilding reproducible output tree..."
  bash "$SCRIPT_DIR/tool/run.sh"
else
  echo "Skipping build step."
fi

[[ -d "$SCRIPT_DIR/out/usr" ]] || {
  echo "Missing built output tree at $SCRIPT_DIR/out/usr" >&2
  exit 1
}

echo "Extracting release archive..."
tar -xzf "$ARCHIVE_PATH" -C "$TMP_DIR"
RELEASE_ROOT="$(find "$TMP_DIR" -mindepth 1 -maxdepth 1 -type d | head -n 1)"

[[ -n "$RELEASE_ROOT" ]] || {
  echo "Release archive did not extract into a top-level directory." >&2
  exit 1
}

echo "Comparing extracted release against reproducible-builds/out..."
# runtime-libs.txt is diagnostic output from ldd and includes load addresses, so ignore it here.
if diff -ruN --no-dereference -x 'runtime-libs.txt' "$RELEASE_ROOT" "$SCRIPT_DIR/out" > "$DIFF_PATH"; then
  echo "OK: release archive matches reproducible-builds/out"
else
  echo "Release archive differs from reproducible-builds/out:" >&2
  cat "$DIFF_PATH" >&2
  exit 1
fi
