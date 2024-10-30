// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#pragma once

#include <exception>
#include <memory>
#include <limits.h>

// Kakadu core includes
#include "kdu_elementary.h"
#include "kdu_messaging.h"
#include "kdu_params.h"
#include "kdu_compressed.h"
#include "kdu_sample_processing.h"
#include "kdu_utils.h" // Access `kdu_memsafe_mul' etc. for safe mem calcs
#include "jp2.h"
#include "jpx.h"
#include "kdu_stripe_decompressor.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/val.h>
#endif

#include "FrameInfo.hpp"
#include "Point.hpp"
#include "Size.hpp"

#define ojph_div_ceil(a, b) (((a) + (b)-1) / (b))

/// <summary>
/// JavaScript API for decoding HTJ2K bistreams with OpenJPH
/// </summary>
class HTJ2KDecoder
{
public:
  /// <summary>
  /// Constructor for decoding a HTJ2K image from JavaScript.
  /// </summary>
  HTJ2KDecoder()
      : pEncoded_(&encodedInternal_),
        pDecoded_(&decodedInternal_)
  {
  }

#ifdef __EMSCRIPTEN__
  /// <summary>
  /// Resizes encoded buffer and returns a TypedArray of the buffer allocated
  /// in WASM memory space that will hold the HTJ2K encoded bitstream.
  /// JavaScript code needs to copy the HTJ2K encoded bistream into the
  /// returned TypedArray.  This copy operation is needed because WASM runs
  /// in a sandbox and cannot access memory managed by JavaScript.
  /// </summary>
  emscripten::val getEncodedBuffer(size_t encodedSize)
  {
    pDecoded_->resize(encodedSize);
    return emscripten::val(emscripten::typed_memory_view(pDecoded_->size(), pDecoded_->data()));
  }

  /// <summary>
  /// Returns a TypedArray of the buffer allocated in WASM memory space that
  /// holds the decoded pixel data
  /// </summary>
  emscripten::val getDecodedBuffer()
  {
    return emscripten::val(emscripten::typed_memory_view(pDecoded_->size(), pDecoded_->data()));
  }
#else
  /// <summary>
  /// Returns the buffer to store the encoded bytes.  This method is not exported
  /// to JavaScript, it is intended to be called by C++ code
  /// </summary>
  std::vector<uint8_t> &getEncodedBytes()
  {
    return *pEncoded_;
  }

  /// <summary>
  /// Sets a pointer to a vector containing the encoded bytes.  This can be used to avoid having to copy the encoded.  Set to 0
  /// to reset to the internal buffer
  /// </summary>
  void setEncodedBytes(std::vector<uint8_t> *pEncoded)
  {
    if (pEncoded == 0)
    {
      pEncoded_ = &encodedInternal_;
    }
    else
    {
      pEncoded_ = pEncoded;
    }
  }

  /// <summary>
  /// Returns the buffer to store the decoded bytes.  This method is not exported
  /// to JavaScript, it is intended to be called by C++ code
  /// </summary>
  const std::vector<uint8_t> &getDecodedBytes() const
  {
    return *pDecoded_;
  }

  /// <summary>
  /// Sets a pointer to a vector containing the encoded bytes.  This can be used to avoid having to copy the encoded.  Set to 0
  /// to reset to the internal buffer
  /// </summary>
  void setDecodedBytes(std::vector<uint8_t> *pDecoded)
  {
    if (pDecoded == 0)
    {
      pDecoded_ = &decodedInternal_;
    }
    else
    {
      pDecoded_ = pDecoded;
    }
  }

#endif

  /// <summary>
  /// Reads the header from an encoded HTJ2K bitstream.  The caller must have
  /// copied the HTJ2K encoded bitstream into the encoded buffer before
  /// calling this method, see getEncodedBuffer() and getEncodedBytes() above.
  /// </summary>
  void readHeader()
  {
    kdu_core::kdu_compressed_source_buffered input(pEncoded_->data(), pEncoded_->size());
    kdu_core::kdu_codestream codestream;
    readHeader_(codestream, input);
    codestream.destroy();
    input.close();
  }

  /// <summary>
  /// Calculates the resolution for a given decomposition level based on the
  /// current values in FrameInfo (which is populated via readHeader() and
  /// decode()).  level = 0 = full res, level = _numDecompositions = lowest resolution
  /// </summary>
  Size calculateSizeAtDecompositionLevel(int decompositionLevel)
  {
    Size result(frameInfo_.width, frameInfo_.height);
    while (decompositionLevel > 0)
    {
      result.width = ojph_div_ceil(result.width, 2);
      result.height = ojph_div_ceil(result.height, 2);
      decompositionLevel--;
    }
    return result;
  }

  /// <summary>
  /// Decodes the encoded HTJ2K bitstream.  The caller must have copied the
  /// HTJ2K encoded bitstream into the encoded buffer before calling this
  /// method, see getEncodedBuffer() and getEncodedBytes() above.
  /// </summary>
  void decode()
  {
    kdu_core::kdu_codestream codestream;
    kdu_core::kdu_compressed_source_buffered input(pEncoded_->data(), pEncoded_->size());
    readHeader_(codestream, input);
    decode_(codestream, input, 0);
    codestream.destroy();
    input.close();
  }

  /// <summary>
  /// Decodes the encoded HTJ2K bitstream to the requested decomposition level.
  /// The caller must have copied the HTJ2K encoded bitstream into the encoded
  /// buffer before calling this method, see getEncodedBuffer() and
  ///  getEncodedBytes() above.
  /// </summary>
  void decodeSubResolution(size_t decompositionLevel)
  {
    kdu_core::kdu_codestream codestream;
    kdu_core::kdu_compressed_source_buffered input(pEncoded_->data(), pEncoded_->size());
    readHeader_(codestream, input);
    decode_(codestream, input, decompositionLevel);
    codestream.destroy();
    input.close();
  }

  /// <summary>
  /// returns the FrameInfo object for the decoded image.
  /// </summary>
  const FrameInfo &getFrameInfo() const
  {
    return frameInfo_;
  }

  /// <summary>
  /// returns the number of wavelet decompositions.
  /// </summary>
  const size_t getNumDecompositions() const
  {
    return numDecompositions_;
  }

  /// <summary>
  /// returns true if the image is lossless, false if lossy
  /// </summary>
  const bool getIsReversible() const
  {
    return isReversible_;
  }

  /// <summary>
  /// returns progression order.
  // 0 = LRCP
  // 1 = RLCP
  // 2 = RPCL
  // 3 = PCRL
  // 4 = CPRL
  /// </summary>
  const size_t getProgressionOrder() const
  {
    return progressionOrder_;
  }

  /// <summary>
  /// returns the down sampling used for component.
  /// </summary>
  Point getDownSample(size_t component) const
  {
    return downSamples_[component];
  }

  /// <summary>
  /// returns the block dimensions
  /// </summary>
  Size getBlockDimensions() const
  {
    return blockDimensions_;
  }

  /// <summary>
  /// returns whether or not a color transform is used
  /// </summary>
  bool getIsUsingColorTransform() const
  {
    return isUsingColorTransform_;
  }

  /// <summary>
  /// returns whether or not HT encoding was used
  /// </summary>
  bool getIsHTEnabled() const
  {
    return isHTEnabled_;
  }

private:
  void readHeader_(kdu_core::kdu_codestream &codestream, kdu_core::kdu_compressed_source_buffered &source)
  {
    kdu_supp::jp2_family_src jp2_ultimate_src;
    jp2_ultimate_src.open(&source);
    kdu_supp::jpx_source jpx_in;
    if (jpx_in.open(&jp2_ultimate_src, true) < 0)
    {
      jp2_ultimate_src.close();
      source.seek(-source.get_pos()); // rewind the source to the first byte since jp2_family_src moved it forward
    }
    else
    {
      // move to the first compositing layer
      kdu_supp::jpx_layer_source jpx_layer = jpx_in.access_layer(0);
    }

    // Create the codestream object.
    codestream.create(&source);

    // Determine number of components to decompress
    kdu_core::kdu_dims dims;
    codestream.get_dims(0, dims);

    int num_components = codestream.get_num_components();
    if (num_components == 2)
      num_components = 1;
    else if (num_components >= 3)
    { // Check that components have consistent dimensions
      num_components = 3;
      kdu_core::kdu_dims dims1;
      codestream.get_dims(1, dims1);
      kdu_core::kdu_dims dims2;
      codestream.get_dims(2, dims2);
      if ((dims1 != dims) || (dims2 != dims))
        num_components = 1;
    }
    codestream.apply_input_restrictions(0, num_components, 0, 0, NULL);
    frameInfo_.width = dims.size.x;
    frameInfo_.height = dims.size.y;
    frameInfo_.componentCount = num_components;
    frameInfo_.bitsPerSample = codestream.get_bit_depth(0);
    frameInfo_.isSigned = codestream.get_signed(0);
  }

  void decode_(kdu_core::kdu_codestream &codestream, kdu_core::kdu_compressed_source_buffered &input, size_t decompositionLevel)
  {
    kdu_core::siz_params *siz = codestream.access_siz();
    kdu_core::kdu_params *cod = siz->access_cluster(COD_params);
    cod->get(Clevels, 0, 0, (int &)numDecompositions_);
    cod->get(Corder, 0, 0, (int &)progressionOrder_);
    cod->get(Creversible, 0, 0, isReversible_);
    cod->get(Cblk, 0, 0, (int &)blockDimensions_.height);
    cod->get(Cblk, 0, 1, (int &)blockDimensions_.width);

    isHTEnabled_ = codestream.get_ht_usage();
    size_t bytesPerPixel = (frameInfo_.bitsPerSample + 1) / 8;
    // Now decompress the image in one hit, using `kdu_stripe_decompressor'
    size_t num_samples = kdu_core::kdu_memsafe_mul(frameInfo_.componentCount,
                                                   kdu_core::kdu_memsafe_mul(frameInfo_.width,
                                                                             frameInfo_.height));
    pDecoded_->resize(num_samples * bytesPerPixel);
    kdu_core::kdu_byte *buffer = pDecoded_->data();
    kdu_supp::kdu_stripe_decompressor decompressor;
    decompressor.start(codestream);
    int stripe_heights[3] = {frameInfo_.height, frameInfo_.height, frameInfo_.height};

    bool is_signed[3] = {frameInfo_.isSigned, frameInfo_.isSigned, frameInfo_.isSigned};
    if (bytesPerPixel == 1)
    {
      decompressor.pull_stripe((kdu_core::kdu_byte *)buffer, stripe_heights);
    }
    else
    {
      decompressor.pull_stripe(
          (kdu_core::kdu_int16 *)buffer,
          stripe_heights,
          NULL,      // sample_offsets
          NULL,      // sample_gaps
          NULL,      // row_gaps
          NULL,      // precisions
          is_signed, // is_signed
          NULL,      // pad_flags
          0          // vectorized_store_prefs
      );
    }
    decompressor.finish();
  }

  std::vector<uint8_t> *pEncoded_;
  std::vector<uint8_t> *pDecoded_;
  std::vector<uint8_t> encodedInternal_;
  std::vector<uint8_t> decodedInternal_;

  // std::vector<uint8_t> encoded_;
  // std::vector<uint8_t> decoded_;
  FrameInfo frameInfo_;
  std::vector<Point> downSamples_;
  size_t numDecompositions_;
  bool isReversible_;
  size_t progressionOrder_;
  Size blockDimensions_;
  bool isUsingColorTransform_;
  bool isHTEnabled_;
};
