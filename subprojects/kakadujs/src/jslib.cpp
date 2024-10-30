// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#include "HTJ2KDecoder.hpp"
#include "HTJ2KEncoder.hpp"

#include <emscripten.h>
#include <emscripten/bind.h>

using namespace emscripten;

static std::string getVersion()
{
  std::string version = KDU_CORE_VERSION;
  return version;
}

EMSCRIPTEN_BINDINGS(charlsjs)
{
  function("getVersion", &getVersion);
}

EMSCRIPTEN_BINDINGS(FrameInfo)
{
  value_object<FrameInfo>("FrameInfo")
      .field("width", &FrameInfo::width)
      .field("height", &FrameInfo::height)
      .field("bitsPerSample", &FrameInfo::bitsPerSample)
      .field("componentCount", &FrameInfo::componentCount)
      .field("isSigned", &FrameInfo::isSigned);
}

EMSCRIPTEN_BINDINGS(Point)
{
  value_object<Point>("Point")
      .field("x", &Point::x)
      .field("y", &Point::y);
}

EMSCRIPTEN_BINDINGS(Size)
{
  value_object<Size>("Size")
      .field("width", &Size::width)
      .field("height", &Size::height);
}

EMSCRIPTEN_BINDINGS(HTJ2KDecoder)
{
  class_<HTJ2KDecoder>("HTJ2KDecoder")
      .constructor<>()
      .function("getEncodedBuffer", &HTJ2KDecoder::getEncodedBuffer)
      .function("getDecodedBuffer", &HTJ2KDecoder::getDecodedBuffer)
      .function("readHeader", &HTJ2KDecoder::readHeader)
      .function("calculateSizeAtDecompositionLevel", &HTJ2KDecoder::calculateSizeAtDecompositionLevel)
      .function("decode", &HTJ2KDecoder::decode)
      .function("decodeSubResolution", &HTJ2KDecoder::decodeSubResolution)
      .function("getFrameInfo", &HTJ2KDecoder::getFrameInfo)
      .function("getDownSample", &HTJ2KDecoder::getDownSample)
      .function("getNumDecompositions", &HTJ2KDecoder::getNumDecompositions)
      .function("getIsReversible", &HTJ2KDecoder::getIsReversible)
      .function("getProgressionOrder", &HTJ2KDecoder::getProgressionOrder)
      .function("getBlockDimensions", &HTJ2KDecoder::getBlockDimensions)
      .function("getIsUsingColorTransform", &HTJ2KDecoder::getIsUsingColorTransform)
      .function("getIsHTEnabled", &HTJ2KDecoder::getIsHTEnabled);
}

EMSCRIPTEN_BINDINGS(HTJ2KEncoder)
{
  class_<HTJ2KEncoder>("HTJ2KEncoder")
      .constructor<>()
      .function("getDecodedBuffer", &HTJ2KEncoder::getDecodedBuffer)
      .function("getEncodedBuffer", &HTJ2KEncoder::getEncodedBuffer)
      .function("encode", &HTJ2KEncoder::encode)
      .function("setDecompositions", &HTJ2KEncoder::setDecompositions)
      .function("setQuality", &HTJ2KEncoder::setQuality)
      .function("setProgressionOrder", &HTJ2KEncoder::setProgressionOrder)
      .function("setBlockDimensions", &HTJ2KEncoder::setBlockDimensions)
      .function("setHTEnabled", &HTJ2KEncoder::setHTEnabled);
}