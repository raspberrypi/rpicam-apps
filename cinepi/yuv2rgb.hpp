/*
 * Copyright (C) 2012 Andre Chen and contributors.
 * andre.hl.chen@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
bool nv21_to_rgb(unsigned char* rgb, unsigned char const* nv21, int width, int height);

template<typename trait>
bool decode_yuv_neon(unsigned char* out, unsigned char const* y, unsigned char const* uv, int width, int height, unsigned char fill_alpha=0xff)
{
    // pre-condition : width, height must be even
    if (0!=(width&1) || width<2 || 0!=(height&1) || height<2 || !out || !y || !uv)
        return false;

    // in & out pointers
    unsigned char* dst = out;

    // constants
    int const stride = width*trait::bytes_per_pixel;
    int const itHeight = height>>1;
    int const itWidth = width>>3;

    uint8x8_t const Yshift = vdup_n_u8(16);
    int16x8_t const half = (int16x8_t)vdupq_n_u16(128);
    int32x4_t const rounding = vdupq_n_s32(128);

    // tmp variable
    uint16x8_t t;

    // pixel block to temporary store 8 pixels
    typename trait::PixelBlock pblock = trait::init_pixelblock(fill_alpha);    

    for (int j=0; j<itHeight; ++j, y+=width, dst+=stride) {
        for (int i=0; i<itWidth; ++i, y+=8, uv+=8, dst+=(8*trait::bytes_per_pixel)) {
            t = vmovl_u8(vqsub_u8(vld1_u8(y), Yshift));
            int32x4_t const Y00 = (const int32x4_t)vmulq_n_u32(vmovl_u16(vget_low_u16(t)), 298);
            int32x4_t const Y01 = (const int32x4_t)vmulq_n_u32(vmovl_u16(vget_high_u16(t)), 298);

            t = vmovl_u8(vqsub_u8(vld1_u8(y+width), Yshift));
            int32x4_t const Y10 = (const int32x4_t)vmulq_n_u32(vmovl_u16(vget_low_u16(t)), 298);
            int32x4_t const Y11 = (const int32x4_t)vmulq_n_u32(vmovl_u16(vget_high_u16(t)), 298);

            // trait::loadvu pack 4 sets of uv into a uint8x8_t, layout : { v0,u0, v1,u1, v2,u2, v3,u3 }
            t = (uint16x8_t)vsubq_s16((int16x8_t)vmovl_u8(trait::loadvu(uv)), half);

            // UV.val[0] : v0, v1, v2, v3
            // UV.val[1] : u0, u1, u2, u3
            int16x4x2_t const UV = vuzp_s16(vget_low_s16((int16x8_t)t), vget_high_s16((int16x8_t)t));

            // tR : 128+409V
            // tG : 128-100U-208V
            // tB : 128+516U
            int32x4_t const tR = vmlal_n_s16(rounding, UV.val[1], 409);
            int32x4_t const tG = vmlal_n_s16(vmlal_n_s16(rounding, UV.val[1], -208), UV.val[0], -100);
            int32x4_t const tB = vmlal_n_s16(rounding, UV.val[0], 516);

            int32x4x2_t const R = vzipq_s32(tR, tR); // [tR0, tR0, tR1, tR1] [ tR2, tR2, tR3, tR3]
            int32x4x2_t const G = vzipq_s32(tG, tG); // [tG0, tG0, tG1, tG1] [ tG2, tG2, tG3, tG3]
            int32x4x2_t const B = vzipq_s32(tB, tB); // [tB0, tB0, tB1, tB1] [ tB2, tB2, tB3, tB3]

            // upper 8 pixels
            trait::store_pixel_block(dst, pblock,
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(R.val[0], Y00)), vqmovun_s32(vaddq_s32(R.val[1], Y01))), 8),
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(G.val[0], Y00)), vqmovun_s32(vaddq_s32(G.val[1], Y01))), 8),
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(B.val[0], Y00)), vqmovun_s32(vaddq_s32(B.val[1], Y01))), 8));

            // lower 8 pixels
            trait::store_pixel_block(dst+stride, pblock,
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(R.val[0], Y10)), vqmovun_s32(vaddq_s32(R.val[1], Y11))), 8),
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(G.val[0], Y10)), vqmovun_s32(vaddq_s32(G.val[1], Y11))), 8),
                    vshrn_n_u16(vcombine_u16(vqmovun_s32(vaddq_s32(B.val[0], Y10)), vqmovun_s32(vaddq_s32(B.val[1], Y11))), 8));
        }
    }
    return true;
}

//------------------------------------------------------------------------------
class NV21toRGB_neon {
public:
    enum { bytes_per_pixel = 3 };
    typedef uint8x8x3_t PixelBlock;
    static PixelBlock const init_pixelblock(unsigned char /*fill_alpha*/) {
        return uint8x8x3_t();
    }
    static uint8x8_t const loadvu(unsigned char const* uv) {
        return vld1_u8(uv);
    }
    static void store_pixel_block(unsigned char* dst, PixelBlock& pblock, uint8x8_t const& r, uint8x8_t const& g, uint8x8_t const& b) {
        pblock.val[0] = r;
        pblock.val[1] = g;
        pblock.val[2] = b;
        vst3_u8(dst, pblock);
    }
};

bool nv21_to_rgb(unsigned char* rgb, unsigned char const* nv21, int width, int height) {
    return decode_yuv_neon<NV21toRGB_neon>(rgb, nv21, nv21+(width*height), width, height);
}

bool nv21_to_rgb(unsigned char* rgb, unsigned char const* y, unsigned char const* uv, int width, int height) {
    return decode_yuv_neon<NV21toRGB_neon>(rgb, y, uv, width, height);
}

//------------------------------------------------------------------------------
class NV21toRGBA_neon {
public:
    enum { bytes_per_pixel = 4 };
    typedef uint8x8x4_t PixelBlock;
    static PixelBlock const init_pixelblock(unsigned char fill_alpha) {
        PixelBlock block;
        block.val[3] = vdup_n_u8(fill_alpha); // alpha channel in the last
        return block;
    }
    static uint8x8_t const loadvu(unsigned char const* uv) {
        return vld1_u8(uv);
    }
    static void store_pixel_block(unsigned char* dst, PixelBlock& pblock, uint8x8_t const& r, uint8x8_t const& g, uint8x8_t const& b) {
        pblock.val[0] = r;
        pblock.val[1] = g;
        pblock.val[2] = b;
        vst4_u8(dst, pblock);
    }
};
bool nv21_to_rgba(unsigned char* rgba, unsigned char alpha, unsigned char const* nv21, int width, int height) {
    return decode_yuv_neon<NV21toRGBA_neon>(rgba, nv21, nv21+(width*height), width, height, alpha);
}

//------------------------------------------------------------------------------
class NV21toBGRA_neon {
public:
    enum { bytes_per_pixel = 4 };
    typedef uint8x8x4_t PixelBlock;
    static PixelBlock const init_pixelblock(unsigned char fill_alpha) {
        PixelBlock block;
        block.val[3] = vdup_n_u8(fill_alpha); // alpha channel in the last
        return block;
    }
    static uint8x8_t const loadvu(unsigned char const* uv) {
        return vld1_u8(uv);
    }
    static void store_pixel_block(unsigned char* dst, PixelBlock& pblock, uint8x8_t const& r, uint8x8_t const& g, uint8x8_t const& b) {
        pblock.val[0] = b;
        pblock.val[1] = g;
        pblock.val[2] = r;
        vst4_u8(dst, pblock);
    }
};
bool nv21_to_bgra(unsigned char* rgba, unsigned char alpha, unsigned char const* nv21, int width, int height) {
    return decode_yuv_neon<NV21toBGRA_neon>(rgba, nv21, nv21+(width*height), width, height, alpha);
}

//------------------------------------------------------------------------------
class NV21toBGR_neon {
public:
    enum { bytes_per_pixel = 3 };
    typedef uint8x8x3_t PixelBlock;
    static PixelBlock const init_pixelblock(unsigned char /*fill_alpha*/) {
        return uint8x8x3_t();
    }
    static uint8x8_t const loadvu(unsigned char const* uv) {
        return vld1_u8(uv);
    }
    static void store_pixel_block(unsigned char* dst, PixelBlock& pblock, uint8x8_t const& r, uint8x8_t const& g, uint8x8_t const& b) {
        pblock.val[0] = b;
        pblock.val[1] = g;
        pblock.val[2] = r;
        vst3_u8(dst, pblock);
    }
};
bool nv21_to_bgr(unsigned char* bgr, unsigned char const* nv21, int width, int height) {
    return decode_yuv_neon<NV21toBGR_neon>(bgr, nv21, nv21+(width*height), width, height);
}

