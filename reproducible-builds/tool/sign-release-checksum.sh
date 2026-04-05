#!/usr/bin/env bash
# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set -euo pipefail

usage() {
  cat <<EOF
Usage:
  $0 --archive /path/to/archive.tar.gz --sha-file /path/to/archive.tar.gz.sha256 \\
     --label LABEL --key KEYID_OR_FPR [--outdir /path/to/output]

Produces:
  <outdir>/<archive>.sha256.<label>.asc

The checksum file must match the archive bytes before signing.
EOF
  exit 1
}

ARCHIVE=""
SHA_FILE=""
LABEL=""
KEY=""
OUTDIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --archive) ARCHIVE="$2"; shift 2 ;;
    --sha-file) SHA_FILE="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    --key) KEY="$2"; shift 2 ;;
    --outdir) OUTDIR="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown arg: $1" >&2; usage ;;
  esac
done

[[ -n "$ARCHIVE" && -n "$SHA_FILE" && -n "$LABEL" && -n "$KEY" ]] || usage
[[ -f "$ARCHIVE" ]] || { echo "Missing archive: $ARCHIVE" >&2; exit 1; }
[[ -f "$SHA_FILE" ]] || { echo "Missing checksum file: $SHA_FILE" >&2; exit 1; }

if [[ "$SHA_FILE" == "$ARCHIVE" ]]; then
  echo "--sha-file must point to the .sha256 file, not the archive itself." >&2
  exit 1
fi

if [[ "$(basename "$SHA_FILE")" != *.sha256 ]]; then
  echo "--sha-file must point to a .sha256 file: $SHA_FILE" >&2
  exit 1
fi

if [[ -z "$OUTDIR" ]]; then
  OUTDIR="$(dirname "$SHA_FILE")"
fi

hash_file() {
  local file_path="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$file_path" | awk '{print $1}'
  else
    shasum -a 256 "$file_path" | awk '{print $1}'
  fi
}

expected_line="$(LC_ALL=C head -n 1 "$SHA_FILE" | tr -d '\r')"
expected_hash="$(printf '%s\n' "$expected_line" | awk '{print $1}')"
expected_name="$(printf '%s\n' "$expected_line" | awk '{print $2}' | sed 's/^\*//')"
actual_hash="$(hash_file "$ARCHIVE")"
actual_name="$(basename "$ARCHIVE")"

[[ -n "$expected_hash" && -n "$expected_name" ]] || {
  echo "Malformed checksum file: $SHA_FILE" >&2
  exit 1
}

if [[ "$expected_name" != "$actual_name" ]]; then
  echo "Checksum file does not refer to $(basename "$ARCHIVE"): $expected_name" >&2
  exit 1
fi

if [[ "$expected_hash" != "$actual_hash" ]]; then
  echo "ERROR: archive SHA mismatch" >&2
  echo "Expected: $expected_hash" >&2
  echo "Actual:   $actual_hash" >&2
  echo "Do NOT sign this checksum file." >&2
  exit 1
fi

mkdir -p "$OUTDIR"
output_sig="$OUTDIR/${actual_name}.sha256.${LABEL}.asc"

echo "SHA check OK. Signing checksum file..."
gpg --batch --yes --armor --detach-sign --local-user "$KEY" \
  -o "$output_sig" "$SHA_FILE"

echo "Created signature: $output_sig"
