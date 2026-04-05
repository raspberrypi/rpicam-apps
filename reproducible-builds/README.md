# Reproducible Builds

This directory contains Secluso's verification tooling for building an exported runtime tree and packaging it into a deterministic release archive for rpicam-apps.

The build and release scripts are in reproducible-builds/tool/.

## Verification Flow

1. Build the exported runtime tree:

```bash
bash reproducible-builds/tool/run.sh
```

That writes the install tree to reproducible-builds/out/.

2. Verify a downloaded GitHub release against a fresh local build:

Drop the release assets into reproducible-builds/release/, then run:

```bash
bash reproducible-builds/verify-reproducible.sh
```

That verifies the checksum, verifies any .asc files present next to the checksum, rebuilds reproducible-builds/out/, extracts the release archive, and diffs it against the local build output.

## Licensing

The Secluso-authored files in this directory tree are separately licensed from the upstream rpicam-apps source tree. See reproducible-builds/LICENSE, reproducible-builds/COPYRIGHT, and reproducible-builds/NOTICE.
