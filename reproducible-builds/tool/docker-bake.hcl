# Copyright (C) 2026 Secluso, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

variable "PLATFORM" {
  default = "linux/arm64"
}

variable "VCS_REF" {
  default = ""
}

variable "SOURCE_DATE_EPOCH" {
  default = "0"
}

variable "OUTPUT_DIR" {
  default = "./reproducible-builds/out"
}

group "default" {
  targets = ["export"]
}

target "export" {
  context    = "."
  dockerfile = "reproducible-builds/tool/Dockerfile"
  target     = "export"
  platforms  = [PLATFORM]
  output     = ["type=local,dest=${OUTPUT_DIR}"]

  args = {
    SOURCE_DATE_EPOCH = SOURCE_DATE_EPOCH
    VCS_REF           = VCS_REF
  }
}
