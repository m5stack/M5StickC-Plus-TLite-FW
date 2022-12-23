// jpge.cpp - C++ class for JPEG compression.
// Public domain, Rich Geldreich <richgel99@gmail.com>
// v1.01, Dec. 18, 2010 - Initial release
// v1.02, Apr. 6, 2011 - Removed 2x2 ordered dither in H2V1 chroma subsampling
// method load_block_16_8_8(). (The rounding factor was 2, when it should have
// been 1. Either way, it wasn't helping.) v1.03, Apr. 16, 2011 - Added support
// for optimized Huffman code tables, optimized dynamic memory allocation down
// to only 1 alloc.
//                        Also from Alex Evans: Added RGBA support, linear
//                        memory allocator (no longer needed in v1.03).
// v1.04, May. 19, 2012: Forgot to set m_pFile ptr to NULL in
// cfile_stream::close(). Thanks to Owen Kaluza for reporting this bug.
//                       Code tweaks to fix VS2008 static code analysis warnings
//                       (all looked harmless). Code review revealed method
//                       load_block_16_8_8() (used for the non-default H2V1
//                       sampling mode to downsample chroma) somehow didn't get
//                       the rounding factor fix from v1.02.

#include "jpge.h"

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <esp_heap_caps.h>

// debug
//  #include <soc/gpio_struct.h>
//  #include <esp_log.h>

#define JPGE_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define JPGE_MIN(a, b) (((a) < (b)) ? (a) : (b))

namespace jpge {

static inline void *jpge_malloc(size_t nSize) {
    void *b = malloc(nSize);
    if (b) {
        return b;
    }
    // check if SPIRAM is enabled and allocate on SPIRAM if allocatable
#if (CONFIG_SPIRAM_SUPPORT && \
     (CONFIG_SPIRAM_USE_CAPS_ALLOC || CONFIG_SPIRAM_USE_MALLOC))
    return heap_caps_malloc(nSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    return NULL;
#endif
}
static inline void jpge_free(void *p) {
    free(p);
}

// Various JPEG enums and tables.
enum {
    M_SOF0 = 0xC0,
    M_DHT  = 0xC4,
    M_SOI  = 0xD8,
    M_EOI  = 0xD9,
    M_SOS  = 0xDA,
    M_DQT  = 0xDB,
    M_APP0 = 0xE0
};
enum {
    DC_LUM_CODES      = 12,
    AC_LUM_CODES      = 256,
    DC_CHROMA_CODES   = 12,
    AC_CHROMA_CODES   = 256,
    MAX_HUFF_SYMBOLS  = 257,
    MAX_HUFF_CODESIZE = 32
};

static const uint8 s_zag[64] = {
    0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};
static const uint8 s_std_lum_quant[64] = {
    16, 11, 12,  14,  12,  10, 16, 14,  13,  14,  18,  17,  16, 19,  24,  40,
    26, 24, 22,  22,  24,  49, 35, 37,  29,  40,  58,  51,  61, 60,  57,  51,
    56, 55, 64,  72,  92,  78, 64, 68,  87,  69,  55,  56,  80, 109, 81,  87,
    95, 98, 103, 104, 103, 62, 77, 113, 121, 112, 100, 120, 92, 101, 103, 99};
static const uint8 s_std_croma_quant[64] = {
    17, 18, 18, 24, 21, 24, 47, 26, 26, 47, 99, 66, 56, 66, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};
static const uint8 s_dc_lum_bits[17]          = {0, 0, 1, 5, 1, 1, 1, 1, 1,
                                        1, 0, 0, 0, 0, 0, 0, 0};
static const uint8 s_dc_lum_val[DC_LUM_CODES] = {0, 1, 2, 3, 4,  5,
                                                 6, 7, 8, 9, 10, 11};
static const uint8 s_ac_lum_bits[17]          = {0, 0, 2, 1, 3, 3, 2, 4,   3,
                                        5, 5, 4, 4, 0, 0, 1, 0x7d};
static const uint8 s_ac_lum_val[AC_LUM_CODES] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
static const uint8 s_dc_chroma_bits[17] = {0, 0, 3, 1, 1, 1, 1, 1, 1,
                                           1, 1, 1, 0, 0, 0, 0, 0};
static const uint8 s_dc_chroma_val[DC_CHROMA_CODES] = {0, 1, 2, 3, 4,  5,
                                                       6, 7, 8, 9, 10, 11};
static const uint8 s_ac_chroma_bits[17] = {0, 0, 2, 1, 2, 4, 4, 3,   4,
                                           7, 5, 4, 4, 0, 1, 2, 0x77};
static const uint8 s_ac_chroma_val[AC_CHROMA_CODES] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

const int YR = 19595, YG = 38470, YB = 7471, CB_R = -11059, CB_G = -21709,
          CB_B = 32768, CR_R = 32768, CR_G = -27439, CR_B = -5329;

static int32 m_last_quality = 0;
static uint8 m_quantization8_tables[2][64];
static int32 m_quantization32_tables[2][64];

static bool m_huff_initialized = false;
static uint m_huff_codes[4][256];
static uint8 m_huff_code_sizes[4][256];
static uint8 m_huff_bits[4][17];
static uint8 m_huff_val[4][256];

static inline uint_fast8_t clamp(int i) {
    return (i < 0) ? 0 : (i > 255 ? 255 : i);
}

static void RGB_to_YCC(uint8 *pDst, const uint8 *pSrc, int num_pixels) {
    for (; num_pixels; pDst += 3, pSrc += 3, num_pixels--) {
        const int r = pSrc[0], g = pSrc[1], b = pSrc[2];
        pDst[0] = static_cast<uint8>((r * YR + g * YG + b * YB + 32768) >> 16);
        pDst[1] = clamp(128 + ((r * CB_R + g * CB_G + b * CB_B + 32768) >> 16));
        pDst[2] = clamp(128 + ((r * CR_R + g * CR_G + b * CR_B + 32768) >> 16));
    }
}

struct RGB565_to_YCC_asm_t {
    const int16 cb_r16    = (-0.168736 * (65536 * 32)) / 31;  // 0
    const int16 cb_g16    = (-0.331264 * (65536 * 32)) / 63;
    const int16 cr_g16    = (-0.418688 * (65536 * 32)) / 63;  // 4
    const int16 cr_b16    = (-0.081312 * (65536 * 32)) / 31;
    const uint16 yr16     = (0.299 * (65536 * 32)) / 31;  // 8
    const uint16 yb16     = (0.114 * (65536 * 32)) / 31;
    const uint16 yg16     = (0.587 * (65536 * 32)) / 63;  // 12
    const uint16 crrcbb16 = (0.5 * (65536 * 32)) / 31;
};

void RGB565_to_YCC_asm(RGB565_to_YCC_asm_t *asm_data, int16 *pDst,
                       const uint16 *pSrc, int num_pixels) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用しない)
        a1 : スタックポインタ     (変更不可)
        a2 : RGB565_to_YCC_asm_t* (変更せずそのまま利用する)
        a3 : int16* pDst          (YCrCb 出力先アドレス
       ループ内で加算しながら利用する) a4 : uint16* pSrc         (RGB565
       入力元アドレス ループ内で加算しながら利用する) a5 : num_pixels
       (ループに設定後 別用途に使用)
    */
    __asm__ __volatile__(
        "l32i.n     a12,a2, 0           \n"
        "l32i.n     a13,a2, 4           \n"
        "l32i.n     a14,a2, 8           \n"
        "l32i.n     a15,a2, 12          \n"
        "wsr        a12,m0              \n"  // special register m0 HIGH:CB_G  ,
                                             // LOW:CB_R
        "wsr        a13,m1              \n"  // special register m1 HIGH:CR_B  ,
                                             // LOW:CR_G
        "wsr        a14,m2              \n"  // special register m2 HIGH:YB    ,
                                             // LOW:YR
        "addi.n     a3, a3, -6          \n"  // a3 -= 3
                                             // ループに入る前に保存先アドレスを3戻す
        "loop       a5, RGB565_to_YCC   \n"  // num_pixelsで指定された回数分のループ開始
        "l16ui      a5, a4, 0           \n"  // a5 = pSrc[0] 元データ16bit読込
                                             // (読込完了まで2サイクル必要)
        "addi.n     a4, a4, 2           \n"  // a4 += 2 pSrcを 2Byte 進める
        "addi.n     a3, a3, 6           \n"  // a3 += 3 pDstを 6Byte 進める

        "extui      a6, a5, 13, 3       \n"  // a6 に G 下3bitを取り出す
        "extui      a7, a5, 0,  3       \n"  // a7 に G 上3bitを取り出す
        "addx8      a6, a7, a6          \n"  // ★a6 = G 6bit (a7 << 3) + a6
        "extui      a7, a5, 8,  5       \n"  // ★a7 = B 5bit
        "extui      a5, a5, 3,  5       \n"  // ★a5 = R 5bit

        "umul.aa.lh a5, a15             \n"  // ACC  = R * CR_R(a15h)
        "mula.da.ll m1, a6              \n"  // ACC += G * CR_G(m1 l)
        "mula.da.hl m1, a7              \n"  // ACC += B * CR_B(m1 h)
        "rsr        a8, ACCLO           \n"  // ★a8 = CR
                                             // (ACCレジスタから計算結果を取得)
        "umul.aa.lh a7, a15             \n"  // ACC  = B * CB_B(a15h)
        "mula.da.hl m0, a6              \n"  // ACC += G * CB_G(m0 h)
        "mula.da.ll m0, a5              \n"  // ACC += R * CB_R(m0 l)
        "rsr        a9, ACCLO           \n"  // ★a9 = CB
                                             // (ACCレジスタから計算結果を取得)
        "umul.aa.ll a6, a15             \n"  // ACC  = G * YG (a15l)
        "mula.ad.ll a5, m2              \n"  // ACC += R * YR (m2 l)
        "mula.ad.lh a7, m2              \n"  // ACC += B * YB (m2 h)
        "rsr        a10,ACCLO           \n"  // ★a10 = Y
                                             // (ACCレジスタから計算結果を取得)

        "addmi      a8, a8, 1024        \n"  // a8 CR += 1024 (丸め誤差調整)
        "addmi      a9, a9, 1024        \n"  // a9 CB += 1024 (丸め誤差調整)
        "srli       a10,a10,11          \n"  // a10 Y >>= 11  右11bitシフト
                                             // ここで 0 ~ 1023程度になる
        "srai       a9, a9, 11          \n"  // a9 CB >>= 11  右11bitシフト
                                             // ここで -512 ~ 512程度になる
        "srai       a8, a8, 11          \n"  // a8 CR >>= 11  右11bitシフト
                                             // ここで -512 ~ 512程度になる
        // "clamps     a9, a9, 9           \n"  // CB = min(511, max(-512, CB))
        // 不要になったのでコメントアウト "clamps     a8, a8, 9           \n" //
        // CR = min(511, max(-512, CR)) 不要になったのでコメントアウト
        "addmi      a10,a10,-512        \n"  // Y -= 512
        "s16i       a10,a3, 0           \n"  // pDst[0] = a10 Y を保存
        "s16i       a9, a3, 2           \n"  // pDst[1] = a9 CB を保存
        "s16i       a8, a3, 4           \n"  // pDst[2] = a8 CR を保存

        "RGB565_to_YCC:                 \n");
}

static void RGB565_to_YCC(uint8 *pDst, const uint16 *pSrc, int num_pixels) {
    // RGB565_to_YCC_asm_t asm_data;
    // RGB565_to_YCC_asm(&asm_data, pDst, pSrc, num_pixels);
    for (; num_pixels; pDst += 3, pSrc++, num_pixels--) {
        uint32 src = pSrc[0];
        int r      = (((src >> 3) & 0x1F) * 0x21) >> 2;
        int g      = src & 0x07;
        g          = (g << 5) + (((src >> 13) & 0x07) << 2) + (g >> 1);
        int b      = (((src >> 8) & 0x1F) * 0x21) >> 2;
        pDst[0] = static_cast<uint8>((r * YR + g * YG + b * YB + 32768) >> 16);
        pDst[1] = clamp(128 + ((r * CB_R + g * CB_G + b * CB_B + 32768) >> 16));
        pDst[2] = clamp(128 + ((r * CR_R + g * CR_G + b * CR_B + 32768) >> 16));
    }
}

static void RGB_to_Y(uint8 *pDst, const uint8 *pSrc, int num_pixels) {
    for (; num_pixels; pDst++, pSrc += 3, num_pixels--) {
        pDst[0] = static_cast<uint8>(
            (pSrc[0] * YR + pSrc[1] * YG + pSrc[2] * YB + 32768) >> 16);
    }
}

static void Y_to_YCC(uint8 *pDst, const uint8 *pSrc, int num_pixels) {
    for (; num_pixels; pDst += 3, pSrc++, num_pixels--) {
        pDst[0] = pSrc[0];
        pDst[1] = 128;
        pDst[2] = 128;
    }
}

// Forward DCT - DCT derived from jfdctint.
enum { CONST_BITS = 13, ROW_BITS = 2 };
#define DCT_DESCALE(x, n) (((x) + (((int32)1) << ((n)-1))) >> (n))
// #define DCT_MUL(var, c) (static_cast<int16>(var) * static_cast<int32>(c))
#define DCT_MUL(var, c) (static_cast<int32>(var) * static_cast<int32>(c))
#define DCT1D(s0, s1, s2, s3, s4, s5, s6, s7)                         \
    int32 t0 = s0 + s7, t7 = s0 - s7, t1 = s1 + s6, t6 = s1 - s6,     \
          t2 = s2 + s5, t5 = s2 - s5, t3 = s3 + s4, t4 = s3 - s4;     \
    int32 t10 = t0 + t3, t13 = t0 - t3, t11 = t1 + t2, t12 = t1 - t2; \
    int32 u1 = DCT_MUL(t12 + t13, 4433);                              \
    s2       = u1 + DCT_MUL(t13, 6270);                               \
    s6       = u1 + DCT_MUL(t12, -15137);                             \
    u1       = t4 + t7;                                               \
    int32 u2 = t5 + t6, u3 = t4 + t6, u4 = t5 + t7;                   \
    int32 z5 = DCT_MUL(u3 + u4, 9633);                                \
    t4       = DCT_MUL(t4, 2446);                                     \
    t5       = DCT_MUL(t5, 16819);                                    \
    t6       = DCT_MUL(t6, 25172);                                    \
    t7       = DCT_MUL(t7, 12299);                                    \
    u1       = DCT_MUL(u1, -7373);                                    \
    u2       = DCT_MUL(u2, -20995);                                   \
    u3       = DCT_MUL(u3, -16069);                                   \
    u4       = DCT_MUL(u4, -3196);                                    \
    u3 += z5;                                                         \
    u4 += z5;                                                         \
    s0 = t10 + t11;                                                   \
    s1 = t7 + u1 + u4;                                                \
    s3 = t6 + u2 + u3;                                                \
    s4 = t10 - t11;                                                   \
    s5 = t5 + u2 + u4;                                                \
    s7 = t4 + u1 + u3;
//*

static int16_t dct2d_tbl[] = {
    0,       //  0
    0,       //  2
    0,       //  4
    0,       //  6
    4433,    //  8
    6270,    // 10
    -15137,  // 12
    9633,    // 14
    -3196,   // 16
    -16069,  // 18
    -20995,  // 20
    -7373,   // 22
    2446,    // 24
    16819,   // 26
    25172,   // 28
    12299,   // 30
};

#define CONST_BIT_STR "13"
#define ROW_BIT_STR   "2"

void dct2d_asm(int32 *q, const int16_t *dct1_tbl) {
    // 関数が呼び出された直後のレジスタの値
    // a0 : リターンアドレス     (使用しない)
    // a1 : スタックポインタ     (変更不可)
    // a2 : q                    (ループ内で加算しながら利用する)
    // a3 : dct2d_tbl            (変更せずそのまま利用する)
    __asm__ __volatile__(
        // "l16si      a4,  a3,  2         \n"     // a4  = (((int32)1) <<
        // ((CONST_BITS-ROW_BITS) - 1))  // DESCALE誤差調整用

        "l32i.n     a15,a3, 20          \n"
        "l32i.n     a14,a3, 16          \n"
        "l32i.n     a13,a3, 12          \n"
        "l32i.n     a12,a3, 8           \n"
        "wsr        a15,m3              \n"  // special register m2 HIGH:-7373 ,
                                             // LOW:-20995
        "wsr        a14,m2              \n"  // special register m2 HIGH:-16069,
                                             // LOW:-3196
        "wsr        a13,m1              \n"  // special register m1 HIGH:9633  ,
                                             // LOW:-15137
        "wsr        a12,m0              \n"  // special register m0 HIGH:6270  ,
                                             // LOW:4433
        "l16si      a7, a3, 30          \n"  // 12299
        "l16si      a6, a3, 28          \n"  // 25172
        "l16si      a5, a3, 26          \n"  // 16819
        "l16si      a4, a3, 24          \n"  // 2446
        "l16si      a3, a3, 22          \n"  // -7373

        "movi.n     a15,8               \n"
        "loop       a15,LOOP_DCT2D_1ST  \n"  // 8回ループ
        "l32i.n     a8 , a2,  0         \n"  // a8   = s0
        "l32i.n     a15, a2,  28        \n"  // a15  = s7
        "l32i.n     a9 , a2,  4         \n"  // a9   = s1
        "l32i.n     a14, a2,  24        \n"  // a14  = s6
        "l32i.n     a10, a2,  8         \n"  // a10  = s2
        "l32i.n     a13, a2,  20        \n"  // a13  = s5
        "l32i.n     a11, a2,  12        \n"  // a11  = s3
        "l32i.n     a12, a2,  16        \n"  // a12  = s4

        "add.n      a15, a8 , a15       \n"  // a15  = t0   (s0 + s7)
        "add.n      a14, a9 , a14       \n"  // a14  = t1   (s1 + s6)
        "add.n      a13, a10, a13       \n"  // a13  = t2   (s2 + s5)
        "add.n      a12, a11, a12       \n"  // a12  = t3   (s3 + s4)
        "subx2      a11, a11, a12       \n"  // a11  = t4   (s3 - s4)
        "subx2      a10, a10, a13       \n"  // a10  = t5   (s2 - s5)
        "subx2      a9,  a9 , a14       \n"  // a9   = t6   (s1 - s6)
        "subx2      a8,  a8 , a15       \n"  // a8   = t7   (s0 - s7)

        "add.n      a12, a15, a12       \n"  // a12  = t10  (t0 + t3)
        "add.n      a13, a14, a13       \n"  // a13  = t11  (t1 + t2)
        "subx2      a14, a14, a13       \n"  // a14  = t12  (t1 - t2)
        "subx2      a15, a15, a12       \n"  // a15  = t13  (t0 - t3)

        "add.n      a13, a12, a13       \n"  // a13  = s0  (t10 + t11)
        "subx2      a12, a12, a13       \n"  // a12  = s4  (t10 - t11)

        // "slli       a13, a13, 2         \n"     // a13  = s0 << ROW_BITS
        // "slli       a12, a12, 2         \n"     // a12  = s4 << ROW_BITS

        "s32i.n     a13, a2,  0         \n"  // save s0
        "s32i.n     a12, a2,  16        \n"  // save s4
        "add.n      a13, a14, a15       \n"  // a13  = u1 = t12 + t13

        "mul.da.ll  m1,  a14            \n"  // a14 = t12 * m1LOW(-15137)
        "rsr        a14, ACCLO          \n"
        "mul.da.hl  m0,  a15            \n"  // a15  = t13 * m0HIGH(6270)
        "rsr        a15, ACCLO          \n"
        "mul.da.ll  m0,  a13            \n"  // a13  = u1 = (a13) * moLOW(4433)
        "rsr        a13, ACCLO          \n"
        "addmi      a13, a13, 4096      \n"  // a13  = u1 + (1 << 12)
                                             // 精度を下げる前の丸め誤差調整

        "add.n      a15, a15, a13       \n"  // a15  = s2 = u1 + DCT_MUL(t13,
                                             // 6270)
        "add.n      a14, a14, a13       \n"  // a14  = s6 = u1 + DCT_MUL(t12,
                                             // -15137)
        "srai       a15, a15, 13        \n"  // a15  = s2 >>
                                             // (CONST_BITS-ROW_BITS)
        "srai       a14, a14, 13        \n"  // a14  = s6 >>
                                             // (CONST_BITS-ROW_BITS)
        "s32i       a15, a2,  8         \n"  // save s2
        "s32i       a14, a2,  24        \n"  // save s6

        "add.n      a12, a10, a8        \n"  // a12  = u4 = t5 + t7
        "add.n      a13, a11, a9        \n"  // a13  = u3 = t4 + t6
        "add.n      a14, a12, a13       \n"  // a14  = z5 = u3 + u4

        "mul.da.hl  m1,  a14            \n"  // a14  = z5 * m1HIGH(9633)
        "rsr        a14, ACCLO          \n"
        "mul.ad.ll  a12, m2             \n"  // a12  = u4 * m2LOW(-3196)
        "rsr        a12, ACCLO          \n"
        "mul.ad.lh  a13, m2             \n"  // a13  = u3 * m2HIGH(-16069)
        "rsr        a13, ACCLO          \n"

        "addmi      a14, a14, 4096      \n"  // a0   = z5 + (1 << 12)
                                             // 精度を下げる前の丸め誤差調整

        "add.n      a12, a12, a14       \n"  // a12  = u4  + z5
        "add.n      a13, a13, a14       \n"  // a13  = u3  + z5
        "add.n      a14, a10, a9        \n"  // a14  = u2 = t5 + t6
        "add.n      a15, a11, a8        \n"  // a15  = u1 = t4 + t7

        "mul.ad.ll  a14, m3             \n"  // a14  = u2 * m3LOW(-20995)
        "rsr        a14, ACCLO          \n"
        "mull       a15, a15, a3        \n"  // a15  = u1 * -7373

        "mull       a11, a11, a4        \n"  // a11  = t4 = DCT_MUL(t4, 2446);
        "mull       a10, a10, a5        \n"  // a10  = t5 = DCT_MUL(t5, 16819);
        "mull       a9,  a9,  a6        \n"  // a9   = t6 = DCT_MUL(t6, 25172);
        "mull       a8,  a8,  a7        \n"  // a8   = t7 = DCT_MUL(t7, 12299);
        "add.n      a11, a11, a15       \n"  // a11 =      t4 += u1
        "add.n      a11, a11, a13       \n"  // a11 = s7 = t4 += u3
        "add.n      a10, a10, a14       \n"  // a10 =      t5 += u2
        "add.n      a10, a10, a12       \n"  // a10 = s5 = t5 += u4
        "add.n      a9,  a9,  a14       \n"  // a9  =      t6 += u2
        "add.n      a9,  a9,  a13       \n"  // a9  = s3 = t6 += u3
        "add.n      a8,  a8,  a15       \n"  // a8  =      t7 += u1
        "add.n      a8,  a8,  a12       \n"  // a8  = s1 = t7 += u4
        "srai       a8,  a8,  13        \n"  // a8  = s1 >>
                                             // (CONST_BITS-ROW_BITS)
        "srai       a9,  a9,  13        \n"  // a9  = s3 >>
                                             // (CONST_BITS-ROW_BITS)
        "srai       a10, a10, 13        \n"  // a10 = s5 >>
                                             // (CONST_BITS-ROW_BITS)
        "srai       a11, a11, 13        \n"  // a11 = s7 >>
                                             // (CONST_BITS-ROW_BITS)
        "s32i.n     a8,  a2,  4         \n"  // save s1
        "s32i.n     a9,  a2,  12        \n"  // save s3
        "s32i.n     a10, a2,  20        \n"  // save s5
        "s32i.n     a11, a2,  28        \n"  // save s7

        "addi.n     a2,  a2, 32         \n"  // 元データを8
                                             // *sizeof(int32_t)進める
        "LOOP_DCT2D_1ST:            \n"

        "addi       a2,  a2, -256       \n"  // 元データ位置をリセット

        // "movi.n     a4,  1              \n"     // // DESCALE誤差調整用
        // "slli       a4,  a4,  17        \n"     // (((int32)1) <<
        // ((CONST_BITS+ROW_BITS+3) - 1)),

        "movi.n     a15,8               \n"
        "loop       a15,LOOP_DCT2D_2ND  \n"  // 8回ループ
        "l32i.n     a8 , a2,  0         \n"  // a8   = s0
        "l32i.n     a15, a2,  224       \n"  // a15  = s7
        "l32i.n     a9 , a2,  32        \n"  // a9   = s1
        "l32i.n     a14, a2,  192       \n"  // a14  = s6
        "l32i.n     a10, a2,  64        \n"  // a10  = s2
        "l32i.n     a13, a2,  160       \n"  // a13  = s5
        "l32i.n     a11, a2,  96        \n"  // a11  = s3
        "l32i.n     a12, a2,  128       \n"  // a12  = s4

        "add.n      a15, a8 , a15       \n"  // a15  = t0   (s0 + s7)
        "add.n      a14, a9 , a14       \n"  // a14  = t1   (s1 + s6)
        "add.n      a13, a10, a13       \n"  // a13  = t2   (s2 + s5)
        "add.n      a12, a11, a12       \n"  // a12  = t3   (s3 + s4)
        "subx2      a11, a11, a12       \n"  // a11  = t4   (s3 - s4)
        "subx2      a10, a10, a13       \n"  // a10  = t5   (s2 - s5)
        "subx2      a9,  a9 , a14       \n"  // a9   = t6   (s1 - s6)
        "subx2      a8,  a8 , a15       \n"  // a8   = t7   (s0 - s7)

        "add.n      a12, a15, a12       \n"  // a12  = t10  (t0 + t3)
        "add.n      a13, a14, a13       \n"  // a13  = t11  (t1 + t2)
        "subx2      a14, a14, a13       \n"  // a14  = t12  (t1 - t2)
        "subx2      a15, a15, a12       \n"  // a15  = t13  (t0 - t3)

        "add.n      a13, a12, a13       \n"  // a13  = s0  (t10 + t11)
        "subx2      a12, a12, a13       \n"  // a12  = s4  (t10 - t11)

        // "addi       a13, a13, 16        \n"     // a13  += 16
        // (DESCALE前の誤差調整) "addi       a12, a12, 16        \n"     // a12
        // += 16 (DESCALE前の誤差調整) "srai       a13, a13, 5         \n" //
        // a13  = s0 DESCALE( ROW_BITS+3 ) "srai       a12, a12, 5         \n"
        // // a12  = s4 DESCALE( ROW_BITS+3 )
        "slli       a13, a13, 13        \n"  // a13 <<= 13
        "slli       a12, a12, 13        \n"  // a12 <<= 13

        "s32i.n     a13, a2,  0         \n"  // save s0
        "s32i       a12, a2,  128       \n"  // save s4
        "add.n      a13, a14, a15       \n"  // a13  = u1 = t12 + t13

        "mul.da.ll  m1,  a14            \n"  // a14 = t12 * m1LOW(-15137)
        "rsr        a14, ACCLO          \n"
        "mul.da.hl  m0,  a15            \n"  // a15  = t13 * m0HIGH(6270)
        "rsr        a15, ACCLO          \n"
        "mul.da.ll  m0,  a13            \n"  // a13  = u1 = (a13) * moLOW(4433)
        "rsr        a13, ACCLO          \n"
        // "add.n      a13, a13, a4        \n"     // a13  = u1 + (a4
        // (精度下げる前に誤差調整分を加算))

        "add.n      a15, a15, a13       \n"  // a15  = s2 = u1 + DCT_MUL(t13,
                                             // 6270)
        "add.n      a14, a14, a13       \n"  // a14  = s6 = u1 + DCT_MUL(t12,
                                             // -15137)
        // "srai       a15, a15, 18        \n"     // a15  = s2 >>
        // (CONST_BITS+ROW_BITS+3) "srai       a14, a14, 18        \n"     //
        // a14  = s6 >> (CONST_BITS+ROW_BITS+3)
        "s32i       a15, a2,  64        \n"  // save s2
        "s32i       a14, a2,  192       \n"  // save s6

        "add.n      a12, a10, a8        \n"  // a12  = u4 = t5 + t7
        "add.n      a13, a11, a9        \n"  // a13  = u3 = t4 + t6
        "add.n      a14, a12, a13       \n"  // a14  = z5 = u3 + u4

        "mul.da.hl  m1,  a14            \n"  // a14  = z5 * m1HIGH(9633)
        "rsr        a14, ACCLO          \n"
        "mul.ad.ll  a12, m2             \n"  // a12  = u4 * m2LOW(-3196)
        "rsr        a12, ACCLO          \n"
        "mul.ad.lh  a13, m2             \n"  // a13  = u3 * m2HIGH(-16069)
        "rsr        a13, ACCLO          \n"

        // "addmi      a14, a14, ????      \n"     // a0   = z5 + (1 << 18)
        // 精度を下げる前の丸め誤差調整

        "add.n      a12, a12, a14       \n"  // a12  = u4  + z5
        "add.n      a13, a13, a14       \n"  // a13  = u3  + z5
        "add.n      a14, a10, a9        \n"  // a14  = u2 = t5 + t6
        "add.n      a15, a11, a8        \n"  // a15  = u1 = t4 + t7

        "mul.ad.ll  a14, m3             \n"  // a14  = u2 * m3LOW(-20995)
        "rsr        a14, ACCLO          \n"
        "mull       a15, a15, a3        \n"  // a15  = u1 * -7373

        "mull       a11, a11, a4        \n"  // a11  = t4 = DCT_MUL(t4, 2446);
        "mull       a10, a10, a5        \n"  // a10  = t5 = DCT_MUL(t5, 16819);
        "mull       a9,  a9,  a6        \n"  // a9   = t6 = DCT_MUL(t6, 25172);
        "mull       a8,  a8,  a7        \n"  // a8   = t7 = DCT_MUL(t7, 12299);

        "add.n      a11, a11, a15       \n"  // a11 =      t4 += u1
        "add.n      a11, a11, a13       \n"  // a11 = s7 = t4 += u3
        "add.n      a10, a10, a14       \n"  // a10 =      t5 += u2
        "add.n      a10, a10, a12       \n"  // a10 = s5 = t5 += u4
        "add.n      a9,  a9,  a14       \n"  // a9  =      t6 += u2
        "add.n      a9,  a9,  a13       \n"  // a9  = s3 = t6 += u3
        "add.n      a8,  a8,  a15       \n"  // a8  =      t7 += u1
        "add.n      a8,  a8,  a12       \n"  // a8  = s1 = t7 += u4
        // "srai       a8,  a8,  18        \n"     // a8  = s1 >>
        // (CONST_BITS+ROW_BITS+3) "srai       a9,  a9,  18        \n"     // a9
        // = s3 >> (CONST_BITS+ROW_BITS+3) "srai       a10, a10, 18        \n"
        // // a10 = s5 >> (CONST_BITS+ROW_BITS+3) "srai       a11, a11, 18 \n"
        // // a11 = s7 >> (CONST_BITS+ROW_BITS+3)
        "s32i.n     a8,  a2,  32        \n"  // save s1
        "s32i       a9,  a2,  96        \n"  // save s3
        "s32i       a10, a2,  160       \n"  // save s5
        "s32i       a11, a2,  224       \n"  // save s7
                                             //*/
        "addi.n     a2,  a2, 4          \n"  // 元データを1
                                             // *sizeof(int32_t)進める
        "LOOP_DCT2D_2ND:            \n");
}
//*/
static void DCT2D(int32 *p) {
    /*
            int32 c, *q = p;
            for (c = 7; c >= 0; c--, q += 8) {
                int32 s0 = q[0], s1 = q[1], s2 = q[2], s3 = q[3], s4 = q[4], s5
    = q[5], s6 = q[6], s7 = q[7]; DCT1D(s0, s1, s2, s3, s4, s5, s6, s7); q[0] =
    s0 << ROW_BITS; q[1] = DCT_DESCALE(s1, CONST_BITS-ROW_BITS); q[2] =
    DCT_DESCALE(s2, CONST_BITS-ROW_BITS); q[3] = DCT_DESCALE(s3,
    CONST_BITS-ROW_BITS); q[4] = s4 << ROW_BITS; q[5] = DCT_DESCALE(s5,
    CONST_BITS-ROW_BITS); q[6] = DCT_DESCALE(s6, CONST_BITS-ROW_BITS); q[7] =
    DCT_DESCALE(s7, CONST_BITS-ROW_BITS);
            }
            for (q = p, c = 7; c >= 0; c--, q++) {
                int32 s0 = q[0*8], s1 = q[1*8], s2 = q[2*8], s3 = q[3*8], s4 =
    q[4*8], s5 = q[5*8], s6 = q[6*8], s7 = q[7*8]; DCT1D(s0, s1, s2, s3, s4, s5,
    s6, s7); q[0*8] = DCT_DESCALE(s0, ROW_BITS+3); q[1*8] = DCT_DESCALE(s1,
    CONST_BITS+ROW_BITS+3); q[2*8] = DCT_DESCALE(s2, CONST_BITS+ROW_BITS+3);
    q[3*8] = DCT_DESCALE(s3, CONST_BITS+ROW_BITS+3); q[4*8] = DCT_DESCALE(s4,
    ROW_BITS+3); q[5*8] = DCT_DESCALE(s5, CONST_BITS+ROW_BITS+3); q[6*8] =
    DCT_DESCALE(s6, CONST_BITS+ROW_BITS+3); q[7*8] = DCT_DESCALE(s7,
    CONST_BITS+ROW_BITS+3);
            }
    /*/
    dct2d_asm(p, dct2d_tbl);
    //*/
}

// Compute the actual canonical Huffman codes/code sizes given the JPEG huff
// bits and val arrays.
static void compute_huffman_table(uint *codes, uint8 *code_sizes, uint8 *bits,
                                  uint8 *val) {
    int i, l, last_p, si;
    static uint8 huff_size[257];
    static uint huff_code[257];
    uint code;

    int p = 0;
    for (l = 1; l <= 16; l++) {
        for (i = 1; i <= bits[l]; i++) {
            huff_size[p++] = (char)l;
        }
    }

    huff_size[p] = 0;
    last_p       = p;  // write sentinel

    code = 0;
    si   = huff_size[0];
    p    = 0;

    while (huff_size[p]) {
        while (huff_size[p] == si) {
            huff_code[p++] = code++;
        }
        code <<= 1;
        si++;
    }

    memset(codes, 0, sizeof(codes[0]) * 256);
    memset(code_sizes, 0, sizeof(code_sizes[0]) * 256);
    for (p = 0; p < last_p; p++) {
        // ESP_LOGE("DEBUG","HUFF_CODE : %d : %d  : %08x", p, huff_size[p],
        // huff_code[p]); codes[val[p]]      = huff_code[p]; code_sizes[val[p]]
        // = huff_size[p];
        codes[val[p]] = huff_code[p] | huff_size[p] << 16;
    }
}

void jpeg_encoder::flush_output_buffer() {
    uint len = JPGE_OUT_BUF_SIZE - m_out_buf_left;
    if (len == 0) return;
    m_out_buf_left = JPGE_OUT_BUF_SIZE;
    m_all_stream_writes_succeeded =
        m_all_stream_writes_succeeded &&
        m_pStream->put_buf(m_out_buf_array[m_out_buf_index], len);
    m_out_buf_index = (m_out_buf_index < JPGE_OUT_BUF_COUNT - 1)
                          ? m_out_buf_index + 1
                          : 0;  // Use buffers in sequence
    m_pOut_buf      = m_out_buf_array[m_out_buf_index];
}

void jpeg_encoder::emit_byte(uint i) {
    *m_pOut_buf++ = i;
    if (--m_out_buf_left == 0) {
        flush_output_buffer();
    }
}

void jpeg_encoder::put_bits(uint bits, uint len) {
    uint32_t bits_in  = m_bits_in - len;
    uint32 bit_buffer = m_bit_buffer + (bits << bits_in);
    if (bits_in <= 24) {
        do {
            uint c = bit_buffer >> 24;
            emit_byte(c);
            if (c == 0xFF) {
                emit_byte(0);
            }
            bit_buffer <<= 8;
            bits_in += 8;
        } while (bits_in <= 24);
    }
    m_bits_in    = bits_in;
    m_bit_buffer = bit_buffer;
}

void jpeg_encoder::emit_word(uint i) {
    // emit_byte(uint8(i >> 8)); emit_byte(uint8(i));
    if (m_out_buf_left < 2) {
        flush_output_buffer();
    }
    auto outbuf = m_pOut_buf;
    outbuf[0]   = i >> 8;
    outbuf[1]   = i;
    outbuf += 2;
    m_pOut_buf = outbuf;
    m_out_buf_left -= 2;
}

// JPEG marker generation.
void jpeg_encoder::emit_marker(uint8 marker) {
    // emit_byte(uint8(0xFF)); emit_byte(uint8(marker));
    if (m_out_buf_left < 2) {
        flush_output_buffer();
    }
    auto outbuf = m_pOut_buf;
    outbuf[0]   = 0xFF;
    outbuf[1]   = marker;
    outbuf += 2;
    m_pOut_buf = outbuf;
    m_out_buf_left -= 2;
}

// Emit JFIF marker
void jpeg_encoder::emit_jfif_app0() {
    emit_marker(M_APP0);
    emit_word(2 + 4 + 1 + 2 + 1 + 2 + 2 + 1 + 1);
    emit_byte(0x4A);
    emit_byte(0x46);
    emit_byte(0x49);
    emit_byte(0x46); /* Identifier: ASCII "JFIF" */
    emit_byte(0);
    emit_byte(1); /* Major version */
    emit_byte(1); /* Minor version */
    emit_byte(0); /* Density unit */
    emit_word(1);
    emit_word(1);
    emit_byte(0); /* No thumbnail image */
    emit_byte(0);
}

// Emit quantization tables
void jpeg_encoder::emit_dqt() {
    for (int i = 0; i < ((m_num_components == 3) ? 2 : 1); i++) {
        emit_marker(M_DQT);
        emit_word(64 + 1 + 2);
        emit_byte(static_cast<uint8>(i));
        for (int j = 0; j < 64; j++) emit_byte(m_quantization8_tables[i][j]);
    }
}

// Emit start of frame marker
void jpeg_encoder::emit_sof() {
    emit_marker(M_SOF0); /* baseline */
    emit_word(3 * m_num_components + 2 + 5 + 1);
    emit_byte(8); /* precision */
    emit_word(m_image_y);
    emit_word(m_image_x);
    emit_byte(m_num_components);
    for (int i = 0; i < m_num_components; i++) {
        emit_byte(static_cast<uint8>(i + 1)); /* component ID     */
        emit_byte((m_comp_h_samp[i] << 4) +
                  m_comp_v_samp[i]); /* h and v sampling */
        emit_byte(i > 0);            /* quant. table num */
    }
}

// Emit Huffman table.
void jpeg_encoder::emit_dht(uint8 *bits, uint8 *val, int index, bool ac_flag) {
    emit_marker(M_DHT);

    int length = 0;
    for (int i = 1; i <= 16; i++) length += bits[i];

    emit_word(length + 2 + 1 + 16);
    emit_byte(static_cast<uint8>(index + (ac_flag << 4)));

    for (int i = 1; i <= 16; i++) emit_byte(bits[i]);

    for (int i = 0; i < length; i++) emit_byte(val[i]);
}

// Emit all Huffman tables.
void jpeg_encoder::emit_dhts() {
    emit_dht(m_huff_bits[0 + 0], m_huff_val[0 + 0], 0, false);
    emit_dht(m_huff_bits[2 + 0], m_huff_val[2 + 0], 0, true);
    if (m_num_components == 3) {
        emit_dht(m_huff_bits[0 + 1], m_huff_val[0 + 1], 1, false);
        emit_dht(m_huff_bits[2 + 1], m_huff_val[2 + 1], 1, true);
    }
}

// emit start of scan
void jpeg_encoder::emit_sos() {
    emit_marker(M_SOS);
    emit_word(2 * m_num_components + 2 + 1 + 3);
    emit_byte(m_num_components);
    for (int i = 0; i < m_num_components; i++) {
        emit_byte(static_cast<uint8>(i + 1));
        if (i == 0)
            emit_byte((0 << 4) + 0);
        else
            emit_byte((1 << 4) + 1);
    }
    emit_byte(0); /* spectral selection */
    emit_byte(63);
    emit_byte(0);
}

void jpeg_encoder::load_block_8_8_grey(int x) {
    // uint8 *pSrc;
    // sample_array_t *pDst = m_sample_array;
    // x <<= 3;
    // for (int i = 0; i < 8; i++, pDst += 8)
    // {
    //     pSrc = m_mcu_lines[i] + x;
    //     pDst[0] = pSrc[0] - 128; pDst[1] = pSrc[1] - 128; pDst[2] = pSrc[2] -
    //     128; pDst[3] = pSrc[3] - 128; pDst[4] = pSrc[4] - 128; pDst[5] =
    //     pSrc[5] - 128; pDst[6] = pSrc[6] - 128; pDst[7] = pSrc[7] - 128;
    // }
}

void load_block_8_8_asm(int32 *pDst, int16 **pSrc, uint32 x) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用せずそのままとする)
        a1 : スタックポインタ     (変更不可)
        a2 : int32* pDst          (変更せずそのまま利用する)
        a3 : uint8** pSrc         (変更せずそのまま利用する)
        a4 : uint32 x             (変更せずそのまま利用する)
    */
    __asm__ __volatile__(
        "movi.n     a15,8               \n"
        "addi       a2, a2, -32         \n"  // 宛先データを8*sizeof(int32)戻す
        "loop       a15,LOOP_BLOCK_8_8  \n"
        "l32i       a5, a3, 0           \n"  // a5 = pSrc  ロード待ち2サイクル
                                             // (最適化のためa5を使うまでに2命令挟む)
        "addi       a3, a3, 4           \n"  // 元データを1進める
        "addi       a2, a2, 32          \n"  // 宛先データを8*sizeof(int32)進める
        "addx2      a5, a4, a5          \n"  // x座標分移動

        "l16si      a8, a5, 0           \n"
        "l16si      a9, a5, 6           \n"
        "l16si      a10,a5, 12          \n"
        "l16si      a11,a5, 18          \n"
        "l16si      a12,a5, 24          \n"
        "l16si      a13,a5, 30          \n"
        "l16si      a14,a5, 36          \n"
        "l16si      a15,a5, 42          \n"
        "s32i.n     a8, a2, 0           \n"
        "s32i.n     a9, a2, 4           \n"
        "s32i.n     a10,a2, 8           \n"
        "s32i.n     a11,a2, 12          \n"
        "s32i.n     a12,a2, 16          \n"
        "s32i.n     a13,a2, 20          \n"
        "s32i.n     a14,a2, 24          \n"
        "s32i.n     a15,a2, 28          \n"

        "LOOP_BLOCK_8_8:            \n");
}

void jpeg_encoder::load_block_8_8(int x, int y, int c) {
    load_block_8_8_asm(m_sample_array, &m_mcu_lines[y << 3], (x * (8 * 3)) + c);
    /*
            uint8 *pSrc;
            sample_array_t *pDst = m_sample_array;
            x = (x * (8 * 3)) + c;
            y <<= 3;
            for (int i = 0; i < 8; i++, pDst += 8)
            {
                pSrc = m_mcu_lines[y + i] + x;
                pDst[0] = pSrc[0 * 3] - 128; pDst[1] = pSrc[1 * 3] - 128;
    pDst[2] = pSrc[2 * 3] - 128; pDst[3] = pSrc[3 * 3] - 128; pDst[4] = pSrc[4 *
    3] - 128; pDst[5] = pSrc[5 * 3] - 128; pDst[6] = pSrc[6 * 3] - 128; pDst[7]
    = pSrc[7 * 3] - 128;
            }
    //*/
}

void load_block_16_8_asm(int32 *pDst, int16 **pSrc, uint32 x) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用せずそのままとする)
        a1 : スタックポインタ     (変更不可)
        a2 : int32* pDst          (変更せずそのまま利用する)
        a3 : uint8** pSrc         (変更せずそのまま利用する)
        a4 : uint32 x             (変更せずそのまま利用する)
    */
    __asm__ __volatile__(
        "movi.n     a15,8               \n"
        "movi.n     a7, 0               \n"  // a7 = a (端数調整成分)
        "movi.n     a8, 2               \n"  // a8 = b (端数調整成分)
        "loop       a15,LOOP_BLOCK_16_8 \n"
        "l32i       a5, a3,  0          \n"  // a5 = pSrc1*
        "l32i       a6, a3,  4          \n"  // a6 = pSrc2*
        "addi       a3, a3,  8          \n"
        "addx2      a5, a4, a5          \n"  // x座標分移動
        "addx2      a6, a4, a6          \n"  // x座標分移動

        "l16si      a12,a5, 0           \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 0           \n"
        "l16si      a14,a5, 6           \n"
        "l16si      a15,a6, 6           \n"
        "add.n      a12,a12,a7          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 0           \n"

        "l16si      a12,a5, 12          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 12          \n"
        "l16si      a14,a5, 18          \n"
        "l16si      a15,a6, 18          \n"
        "add.n      a12,a12,a8          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 4           \n"

        "l16si      a12,a5, 24          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 24          \n"
        "l16si      a14,a5, 30          \n"
        "l16si      a15,a6, 30          \n"
        "add.n      a12,a12,a7          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 8           \n"

        "l16si      a12,a5, 36          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 36          \n"
        "l16si      a14,a5, 42          \n"
        "l16si      a15,a6, 42          \n"
        "add.n      a12,a12,a8          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 12          \n"

        "l16si      a12,a5, 48          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 48          \n"
        "l16si      a14,a5, 54          \n"
        "l16si      a15,a6, 54          \n"
        "add.n      a12,a12,a7          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 16          \n"

        "l16si      a12,a5, 60          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 60          \n"
        "l16si      a14,a5, 66          \n"
        "l16si      a15,a6, 66          \n"
        "add.n      a12,a12,a8          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 20          \n"

        "l16si      a12,a5, 72          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 72          \n"
        "l16si      a14,a5, 78          \n"
        "l16si      a15,a6, 78          \n"
        "add.n      a12,a12,a7          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 24          \n"

        "l16si      a12,a5, 84          \n"  // a12~a15に4ピクセル取得
        "l16si      a13,a6, 84          \n"
        "l16si      a14,a5, 90          \n"
        "l16si      a15,a6, 90          \n"
        "add.n      a12,a12,a8          \n"  // ピクセル値合算
        "add.n      a12,a12,a13         \n"
        "add.n      a12,a12,a14         \n"
        "add.n      a12,a12,a15         \n"
        "srai       a12,a12,2           \n"  // a12 2ビット右シフト(符号付)
        "s32i.n     a12,a2, 28          \n"

        "addi       a2, a2, 32          \n"  // a2 pDst += 8
        "mov        a9, a7              \n"  // 端数調整成分をスワップ
        "mov        a7, a8              \n"
        "mov        a8, a9              \n"

        "LOOP_BLOCK_16_8:           \n");
}

void jpeg_encoder::load_block_16_8(int x, int c) {
    // debug
    //  #ifdef _SOC_GPIO_STRUCT_H_
    //  GPIO.out1_w1ts.val = 1;
    //  #endif
    //*
    load_block_16_8_asm(m_sample_array, m_mcu_lines, (x * (16 * 3)) + c);
    /*/
            uint8 *pSrc1, *pSrc2;
            sample_array_t *pDst = m_sample_array;
            x = (x * (16 * 3)) + c;
            int a = 0, b = 2;
            for (int i = 0; i < 16; i += 2, pDst += 8)
            {
                pSrc1 = m_mcu_lines[i + 0] + x;
                pSrc2 = m_mcu_lines[i + 1] + x;
                pDst[0] = ((pSrc1[ 0 * 3] + pSrc1[ 1 * 3] + pSrc2[ 0 * 3] +
    pSrc2[ 1 * 3] + a) >> 2) - 128; pDst[1] = ((pSrc1[ 2 * 3] + pSrc1[ 3 * 3] +
    pSrc2[ 2 * 3] + pSrc2[ 3 * 3] + b) >> 2) - 128; pDst[2] = ((pSrc1[ 4 * 3] +
    pSrc1[ 5 * 3] + pSrc2[ 4 * 3] + pSrc2[ 5 * 3] + a) >> 2) - 128; pDst[3] =
    ((pSrc1[ 6 * 3] + pSrc1[ 7 * 3] + pSrc2[ 6 * 3] + pSrc2[ 7 * 3] + b) >> 2) -
    128; pDst[4] = ((pSrc1[ 8 * 3] + pSrc1[ 9 * 3] + pSrc2[ 8 * 3] + pSrc2[ 9 *
    3] + a) >> 2) - 128; pDst[5] = ((pSrc1[10 * 3] + pSrc1[11 * 3] + pSrc2[10 *
    3] + pSrc2[11 * 3] + b) >> 2) - 128; pDst[6] = ((pSrc1[12 * 3] + pSrc1[13 *
    3] + pSrc2[12 * 3] + pSrc2[13 * 3] + a) >> 2) - 128; pDst[7] = ((pSrc1[14 *
    3] + pSrc1[15 * 3] + pSrc2[14 * 3] + pSrc2[15 * 3] + b) >> 2) - 128; int
    temp = a; a = b; b = temp;
            }
    //*/
    // debug
    //  #ifdef _SOC_GPIO_STRUCT_H_
    //  GPIO.out1_w1tc.val = 1;
    //  #endif
}

void jpeg_encoder::load_block_16_8_8(int x, int c) {
    // uint8 *pSrc1;
    // sample_array_t *pDst = m_sample_array;
    // x = (x * (16 * 3)) + c;
    // for (int i = 0; i < 8; i++, pDst += 8)
    // {
    //     pSrc1 = m_mcu_lines[i + 0] + x;
    //     pDst[0] = ((pSrc1[ 0 * 3] + pSrc1[ 1 * 3]) >> 1) - 128; pDst[1] =
    //     ((pSrc1[ 2 * 3] + pSrc1[ 3 * 3]) >> 1) - 128; pDst[2] = ((pSrc1[ 4 *
    //     3] + pSrc1[ 5 * 3]) >> 1) - 128; pDst[3] = ((pSrc1[ 6 * 3] + pSrc1[ 7
    //     * 3]) >> 1) - 128; pDst[4] = ((pSrc1[ 8 * 3] + pSrc1[ 9 * 3]) >> 1) -
    //     128; pDst[5] = ((pSrc1[10 * 3] + pSrc1[11 * 3]) >> 1) - 128; pDst[6]
    //     = ((pSrc1[12 * 3] + pSrc1[13 * 3]) >> 1) - 128; pDst[7] = ((pSrc1[14
    //     * 3] + pSrc1[15 * 3]) >> 1) - 128;
    // }
}

void load_quantized_coefficients_asm(int16 *pDst, const int32 *pSrc,
                                     const int32 *q_tbl, const uint8 *zag) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用せずそのままとする)
        a1 : スタックポインタ     (変更不可)
        a2 : int16* pDst          (ループ中で加算しながら利用する)
        a3 : const int32* pSrc    (変更せずそのまま利用する)
        a4 : const int32* q_tbl   (ループ中で加算しながら利用する)
        a5 : const uint8* zag     (ループ中で加算しながら利用する)
    */
    __asm__ __volatile__(
        "movi.n     a15,32                  \n"  //
        "loop       a15,LOOP_QUANT_COEFF    \n"  // 32回ループ
                                                 // (処理自体は64回行う。1ループで2ピクセル処理するので半分でよい)
        "l8ui       a15,a5,  0              \n"  // a15 = zag[0]
        "l8ui       a14,a5,  1              \n"  // a14 = zag[1]
        // "l8ui       a13,a5,  2              \n"     // a13 = zag[2]
        // "l8ui       a12,a5,  3              \n"     // a12 = zag[3]

        "l32i.n     a8, a4, 0               \n"  // a8 = quant0 = q_tbl[0]
        "addx4      a15,a15,a3              \n"  // a15 = &pSrc[zag[0]]
        "l32i.n     a15,a15,0               \n"  // a15 = src0 = pSrc[zag[0]]
        "srai       a7, a8, 1               \n"  // a7 = a8 >> 1
        "neg        a6, a7                  \n"  // a6  = -a7
        "movltz     a7, a6, a15             \n"  // if (src0 < 0) a7 = -a7
        "add.n      a15,a15,a7              \n"  // a15 src0 += a7
        "quos       a15,a15,a8              \n"  // a15 = a15 / quant0

        "l32i.n     a8, a4, 4               \n"  // a8 = quant1 = q_tbl[1]
        "addx4      a14,a14,a3              \n"  // a14 = &pSrc[zag[1]]
        "l32i.n     a14,a14,0               \n"  // a14 = src1 = pSrc[zag[1]]
        "srai       a7, a8, 1               \n"  // a7 = a8 >> 1
        "neg        a6, a7                  \n"  // a6  = -a7
        "movltz     a7, a6, a14             \n"  // if (src1 < 0) a7 = -a7
        "add.n      a14,a14,a7              \n"  // a14 src1 += a7
        "quos       a14,a14,a8              \n"  // a14 = a14 / quant1

        // "l32i.n     a8, a4, 8               \n"     // a8 = quant2 = q_tbl[2]
        // "addx4      a13,a13,a3              \n"     // a13 = &pSrc[zag[2]]
        // "l32i       a13,a13,0               \n"     // a13 = src0 =
        // pSrc[zag[2]] "srai       a7, a8, 1               \n"     // a7 = a8
        // >> 1 "neg        a6, a7                  \n"     // a6  = -a7 "movltz
        // a7, a6, a13             \n"     // if (src0 < 0) a7 = -a7 "add.n
        // a13,a13,a7              \n"     // a13 src0 += a7 "quos a13,a13,a8
        // \n"     // a13 = a13 / quant0

        // "l32i.n     a8, a4, 12              \n"     // a8 = quant3 = q_tbl[3]
        // "addx4      a12,a12,a3              \n"     // a12 = &pSrc[zag[3]]
        // "l32i       a12,a12,0               \n"     // a12 = src1 =
        // pSrc[zag[3]] "srai       a7, a8, 1               \n"     // a7 = a8
        // >> 1 "neg        a6, a7                  \n"     // a6  = -a7 "movltz
        // a7, a6, a12             \n"     // if (src1 < 0) a7 = -a7 "add.n
        // a12,a12,a7              \n"     // a12 src1 += a7 "quos a12,a12,a8
        // \n"     // a12 = a12 / quant1

        "addi.n     a5, a5, 2               \n"  // zag を2つ進める
        "addi.n     a4, a4, 8               \n"  // q_tbl を2つ進める
                                                 // (*sizeof(int32))
        "s16i       a15,a2, 0               \n"  // pDst[0] に結果を保存
        "s16i       a14,a2, 2               \n"  // pDst[1] に結果を保存
        // "s16i       a13,a2, 4               \n"     // pDst[2] に結果を保存
        // "s16i       a12,a2, 6               \n"     // pDst[3] に結果を保存
        "addi.n     a2, a2, 4               \n"  // pDst を2つ進める
                                                 // (*sizeof(int16))
        "LOOP_QUANT_COEFF:               \n");
}

void jpeg_encoder::load_quantized_coefficients(int component_num) {
    load_quantized_coefficients_asm(m_coefficient_array, m_sample_array,
                                    m_quantization32_tables[component_num > 0],
                                    s_zag);
    /* // 原型
            int32 *q = m_quantization32_tables[component_num > 0];
            int16 *pDst = m_coefficient_array;
            const uint8 *zag = s_zag;
            auto sample = m_sample_array;
            for (int i = 0; i < 64; i++, ++zag)
            {
                sample_array_t j = sample[*zag];
                if (j < 0)
                {
                    if ((j = -j + (*q >> 1)) < *q)
                        *pDst++ = 0;
                    else
                        *pDst++ = static_cast<int16>(-(j / *q));
                }
                else
                {
                    if ((j = j + (*q >> 1)) < *q)
                        *pDst++ = 0;
                    else
                        *pDst++ = static_cast<int16>((j / *q));
                }
                q++;
            }
    // // C++最適化
            for (int i = 0; i < 32; i++, pDst += 2)
            {
                auto z0 = zag[0];
                auto z1 = zag[1];
                int32 quant0 = *q++;
                int32 quant1 = *q++;
                sample_array_t j0 = sample[z0];
                sample_array_t j1 = sample[z1];
                int32 tmp0 = (quant0 >> 1);
                int32 tmp1 = (quant1 >> 1);
                if (j0 < 0) { tmp0 = -tmp0; }
                if (j1 < 0) { tmp1 = -tmp1; }
                tmp0 = (tmp0 + j0) / quant0;
                tmp1 = (tmp1 + j1) / quant1;
                pDst[0] = tmp0;
                zag += 2;
                pDst[1] = tmp1;
            }
    //*/
}

struct ccpt_asm_t {
    uint32_t a0;                       // 0
    uint32_t dummy;                    // 4
    uint8 **pOut_buf;                  // 8  出力データバッファ
    uint32 *out_buf_left;              // 12 出力データバッファ残量
    uint32 bit_buffer;                 // 16 出力ビットバッファ
    uint32 bits_in;                    // 20 出力ビットバッファ残量
    const uint32 *codes_0;             // 24
    const uint32 *codes_1;             // 28
    const uint8 *code_sizes_0;         // 32
    const uint8 *code_sizes_1;         // 36
    jpeg_encoder *jpe;                 // 40
    void (jpeg_encoder::*fp_flush)();  // 44
    uint32_t buffer0;                  // 48
    uint32_t buffer1;
    uint32_t buffer2;
    uint32_t buffer3;
    uint32_t buffer4;
    uint32_t buffer5;
    uint32_t buffer6;
    uint32_t buffer7;
};

void put_bits_asm_16(uint8_t **dst, uint32 *out_buf_left, uint32 *data) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用せずそのままとする)
        a1 : スタックポインタ     (変更不可)
        a2 : uint32 data          (変更せずそのまま利用する)
        a3 : const ccpt_asm_t*    (変更せずそのまま利用する)
        a4 : uint32* out_buf_left (ポインタ先の値の減算処理)
    */
    __asm__ __volatile__(
        "l32i.n     a9, a4, 0               \n"  // a9 = 元データアドレス読込
        "l32i.n     a10,a2, 0               \n"  // a10 = 出力先 dst読込
        "l32i.n     a11,a3, 0               \n"  // a11 = 出力バッファ残量
                                                 // out_buf_left読込
        "srli       a12,a9, 24              \n"  // a12 = データ1件目8bit取得
        "extui      a13,a9, 16, 8           \n"  // a13 = データ2件目8bit取得
        "movi.n     a14,1                   \n"
        "movi.n     a15,2                   \n"
        "s16i       a12,a10,0               \n"  // a12
                                                 // をバッファに出力(意図的に2Byte出力し後続データを0化
                                                 // (アライメント違反の可能性があるがESP32はこれが可能))
        "addi       a12,a12,-255            \n"  // a12 から 255引く
        "moveqz     a14,a15,a12             \n"  // a12
                                                 // がゼロの場合はa14を2にする
        "add.n      a10,a10,a14             \n"  // a10 出力先バッファを進める
        "sub        a11,a11,a14             \n"  // a11 バッファ残量を減らす

        "movi.n     a14,1                   \n"
        "s16i       a13,a10,0               \n"  // a13
                                                 // をバッファに出力(意図的に2Byte出力し後続データを0化
        "addi       a13,a13,-255            \n"  // a13 から 255引く
        "moveqz     a14,a15,a13             \n"  // a14 = (a13が0 データ == 255)
                                                 // ? 2 : 1
        "add.n      a10,a10,a14             \n"  // a10 出力先バッファを進める
        "sub        a11,a11,a14             \n"  // a11 バッファ残量を減らす

        "slli       a9, a9, 16              \n"  // a9 =
                                                 // データを左16ビットシフト
        "s32i.n     a9, a4, 0               \n"  // データを書き戻す
        "s32i.n     a10,a2, 0               \n"  // バッファ残量をメモリに書き戻す
        "s32i.n     a11,a3, 0               \n"  // バッファアドレスをメモリに書き戻す
                                                 //*/
    );
}

void flush_output_buffer_asm(ccpt_asm_t *s) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (使用せずそのままとする)
        a1 : スタックポインタ     (変更不可)
        a2 : ccpt_asm_t*          (変更せずそのまま利用する)
    */
    __asm__ __volatile__(
        "l32i       a11,a2, 44              \n"  // a11に関数ポインタ読込
        "l32i       a10,a2, 40              \n"  // a10にjpegクラスのポインタ読込
        "callx8     a11                     \n"  // 関数呼び出し
    );
}

void code_coefficients_pass_two_asm(ccpt_asm_t *s, const int16 *pSrc,
                                    int32 *last_dc_val) {
    /* 関数が呼び出された直後のレジスタの値
        a0 : リターンアドレス     (退避して別用途に使用)
        a1 : スタックポインタ     (変更不可)
        a2 : ccpt_asm_t*          (変更せずそのまま利用する)
        a3 : const int16* pSrc    (変更せずそのまま利用する)
        a4 : int32* last_dc_val   (利用後、別用途に使用)

        a9 : -1 固定値
        a10:保存先アドレス
        a11:保存先バッファ残量
        a12:32bitバッファ
        a13:32bitバッファ残ビット数
    */
    __asm__ __volatile__(

        "s32i.n     a0, a2, 0               \n"  // a0 を保存
        "l16si      a5, a3, 0               \n"  // a5 = pSrc[0]読み込み
        "l32i.n     a6, a4, 0               \n"  // a6 = last_dc_val 読み込み
        "l32i.n     a15,a2, 8               \n"  // a15 = **pOut_buf
        "l32i.n     a11,a2, 12              \n"  // a11 = *out_buf_left
        "l32i.n     a12,a2, 16              \n"  // a12 = bit_buffer
        "l32i.n     a13,a2, 20              \n"  // a13 = bits_in
        "l32i.n     a14,a2, 24              \n"  // a14 = s.codes_0
        "l32i.n     a15,a15,0               \n"  // a15 = *pOut_buf
        "l32i.n     a11,a11,0               \n"  // a11 = out_buf_left
        "movi.n     a10,-1                  \n"  // a10に-1をセット
        "s32i.n     a5, a4, 0               \n"  // *last_dc_val に
                                                 // pSrc[0]を保存
        "sub        a5, a5, a6              \n"  // a5 = pSrc[0] - last_dc_val;
                                                 // // DC成分減算処理

        "bnez       a5, LABEL_CCPT_DC_NOT_0 \n"  // DC成分が0でなければジャンプ

        // DC成分が0の時の処理
        "l16ui      a6, a14,2               \n"  // a6 = len    code_sizes_0[0]
                                                 // の値
        "l16ui      a5, a14,0               \n"  // a5 = code   codes_0[0] の値
        "j          LABEL_CCPT_DC_OUTPUT    \n"  // a5とa6に値をセットして次の処理へ進む

        // DC成分が0でない時の処理
        "LABEL_CCPT_DC_NOT_0: \n"

        "abs        a6, a5                  \n"  // a5の符号化に必要なビット数を求め、a6に代入する
        "nsau       a6, a6                  \n"  // a6 = 32 -
                                                 // 符号化に必要なビット数
        "bgez       a5,LABEL_CCPT_POSIVAL_DC \n"

        // if (a5 < 0) { a5 = (a5 - 1) & (-1 >> a6); }
        // JPEGエンコード時、マイナス値は1減らした上で必要なビット数のみにマスクする必要がある
        // "movi.n     a9, -1                  \n"
        "ssr        a6                      \n"  // SARに右シフト量を設定
        "srl        a9, a10                 \n"  // a9 = (~0u >> a6)
                                                 // 右シフト処理
        "add.n      a5, a5, a9              \n"  // これをa5に加算することで、1減算処理とマスク処理が同時に完了する

        "LABEL_CCPT_POSIVAL_DC:             \n"
        "addi.n     a6, a6,-32              \n"
        "neg        a6, a6                  \n"  // a6 = nbits;
        "addx4      a7, a6, a14             \n"  // a7 = &codes_0[nbits]
        "l16ui      a8, a7, 2               \n"  // a8 = len  codes_0[nbits]
                                                 // の上位2バイト
        "l16ui      a7, a7, 0               \n"  // a7 = code codes_0[nbits]
                                                 // の下位2バイト

        // 符号化に必要な情報を32bitバッファに反映
        "bge        a13,a8, LABEL_CCPT_SKIP0\n"  // 32bitバッファが不足していれば出力
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP0: \n"
        "sub        a13,a13,a8              \n"  // bits_in -= len
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a7, a7                  \n"  // code <<= bits_in
        "add.n      a12,a12,a7              \n"  // bit_buffer += code

        "LABEL_CCPT_DC_OUTPUT:              \n"  // DC成分が0の場合の処理が終わった後のジャンプ先
        // a6データを32bitバッファに反映
        "bge        a13,a6, LABEL_CCPT_SKIP1\n"
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP1: \n"
        "sub        a13,a13,a6              \n"  // bits_in -= nbits
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a5, a5                  \n"  // data <<= bits_in
        "add.n      a12,a12,a5              \n"  // bit_buffer += code

        /// ここまででDC成分の出力が完了、ここからAC成分の出力(63回ループ)-------------

        "l32i.n     a14,a2, 28              \n"  // a14 = s.codes_1
        "movi.n     a4, 63                  \n"  // a4 = ループカウンタ

        "bgei       a11,8, LABEL_CCPT_ACLOOP2 \n"

        "LABEL_CCPT_ACLOOP: \n"  // バッファ残量不足時のデータ送信処理

        "l32i.n     a10,a2, 8               \n"  // a10 = **pOut_buf
        "l32i.n     a0, a2, 12              \n"  // a0  = *out_buf_left
        "l32i.n     a9, a2, 44              \n"  // a9
                                                 // にバッファ出力関数のポインタ読込
        "s32i.n     a15,a10,0               \n"  // s.pOut_buf = a15
        "l32i.n     a10,a2, 40              \n"  // a10にjpegクラスのポインタ読込
        "s32i.n     a11,a0, 0               \n"  // s.out_buf_left = a11

        "callx8     a9                      \n"  // 関数呼び出し

        "l32i.n     a15,a2, 8               \n"  // a15 = **pOut_buf
        "l32i.n     a11,a2, 12              \n"  // a11 = *out_buf_left
        "movi.n     a10,-1                  \n"  // a10に-1をセット
        "l32i.n     a15,a15,0               \n"  // a15 = *pOut_buf
        "l32i.n     a11,a11,0               \n"  // a11 = out_buf_left

        "LABEL_CCPT_ACLOOP2: \n"

        // pSrcを順に調べ、0が連続する回数をa6にカウント。
        // 0でない値が見つかったら後続の処理に進む。
        "mov.n      a6, a4                  \n"  // a6 = a4
                                                 // run_lenを取得する準備
        "beqz       a4, LABEL_CCPT_EXIT     \n"  // ループ回数が0に達していたら終了処理に進む
        "addi.n     a6, a6, -1              \n"  // --a6  run_lenを1減らす
        "loop       a4, LABEL_LOOP_RUNLEN   \n"  // 0以外のデータを探すループ開始
        "l16si      a5, a3, 2               \n"  // a5 = pSrc[1];
        "addi.n     a3, a3, 2               \n"  // ++pSrc;
        "addi.n     a4, a4, -1              \n"  // --a4;
        "bnez       a5, LABLE_AC_FOUND      \n"  // if (a5 != 0) goto AC_FOUND;
        "LABEL_LOOP_RUNLEN: \n"
        "addi.n     a6, a6, 1               \n"  // ++a6  run_lenを1増やす
        "LABEL_CCPT_EXIT: \n"

        // run_lenが0なら処理を省略しLABEL_CCPT_EXIT2へ進む
        "beq        a6, a4, LABEL_CCPT_EXIT2\n"  // if (run_len == i) { goto
                                                 // LABEL_CCPT_RETURN }
        "l16ui      a6, a14,2               \n"  // a6 = len    codes_1[0]
                                                 // の上位2バイト
        "l16ui      a5, a14,0               \n"  // a5 = code   codes_1[0]
                                                 // の下位2バイト
        // データ本体を32bitバッファに反映
        "bge        a13,a6, LABEL_CCPT_SKIP5\n"  // 32bitバッファが不足していれば出力
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP5: \n"
        "sub        a13,a13,a6              \n"  // bits_in -= len
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a5, a5                  \n"  // code <<= bits_in
        "add.n      a12,a12,a5              \n"  // bit_buffer += code

        "bgei       a13,16,LABEL_CCPT_EXIT2\n"  // 32bitバッファにデータが2Byte以上あれば出力
        "call0      LABEL_BIT_PUT_SUB       \n"

        "LABEL_CCPT_EXIT2: \n"
        "l32i.n     a8, a2, 8               \n"  // a8 = **pOut_buf
        "l32i.n     a9, a2, 12              \n"  // a9 = *out_buf_left
        "l32i.n     a0, a2, 0               \n"  // a0 = リターンアドレスを復元
        "s32i.n     a15,a8, 0               \n"  // a15 = *pOut_bufの値を保存
        "s32i.n     a11,a9, 0               \n"  // a11 = out_buf_leftの値を保存
        "s32i.n     a12,a2, 16              \n"  // a12 = 32ビットバッファ保存
        "s32i.n     a13,a2, 20              \n"  // a13 =
                                                 // 32ビットバッファ残量保存
        "retw.n                             \n"  // 関数終了

        "LABLE_AC_FOUND: \n"
        "sub        a6, a6, a4              \n"  // run_len -= i

        // run_lenが16未満なら処理を省略
        "blti       a6, 16, LABEL_CCPT_EXIT_RLT16\n"  // if (run_len < 16) goto
                                                      // RLT16

        "l16ui      a8, a14, 0xF0*4+2       \n"  // a8 = size
                                                 // &code_sizes_1[0xF0]
        "l16ui      a7, a14, 0xF0*4         \n"  // a7 = code  &codes_1[0xF0]

        "LABEL_CCPT_LOOP_RLT16: \n"
        "addi.n     a6, a6, -16             \n"  // run_len -= 16;
        "bge        a13,a8, LABEL_CCPT_SKIP2\n"  // 32bitバッファが不足していれば出力
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP2: \n"
        "sub        a13,a13,a8              \n"  // bits_in -= a8
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a0, a7                  \n"       // code <<= bits_in
        "add.n      a12,a12,a0              \n"       // bit_buffer += code
        "blti       a6, 16, LABEL_CCPT_EXIT_RLT16\n"  // if (run_len >= 16) goto
                                                      // ENTER_RLT16
        "addi.n     a6, a6, -16             \n"       // run_len -= 16;
        "bge        a13,a8, LABEL_CCPT_SKIP2\n"
        "call0      LABEL_BIT_PUT_SUB       \n"
        "j          LABEL_CCPT_SKIP2\n"

        "LABEL_CCPT_EXIT_RLT16: \n"

        "slli       a0, a6, 4               \n"  // a0 = run_len << 4;

        "abs        a6, a5                  \n"  // a6 = abs(a5)
        "nsau       a6, a6                  \n"  // a6 = 32 -
                                                 // (a5の符号化に必要なビット数)
        "bgez       a5,LABEL_CCPT_POSIVAL_AC \n"  // 正の値の場合はマスク処理を省略する

        // if (a5 < 0) { a5 = (a5 - 1) & (-1 >> a6); }
        // JPEGエンコード時、マイナス値は1減らした上で必要なビット数のみにマスクする必要がある
        // "movi.n     a9, -1                  \n"
        "ssr        a6                      \n"  // SARに右シフト量を設定
        "srl        a9, a10                 \n"  // a9 = (~0u >> a6)
                                                 // 右シフト処理
        "add.n      a5, a5, a9              \n"  // これをa5に加算することで、1減算処理とマスク処理が同時に完了する

        "LABEL_CCPT_POSIVAL_AC:             \n"
        "addi.n     a6, a6,-32              \n"
        "sub        a0, a0, a6              \n"  // a0 = nbits + (run_len << 4)
        "addx4      a7, a0, a14             \n"  // a7 = codes_1 + (run_len <<
                                                 // 4)
        "l16ui      a8, a7, 2               \n"  // a8 = size codes_1[run_len <<
                                                 // 4 | nbits] の上位2バイト
        "l16ui      a7, a7, 0               \n"  // a7 = code codes_1[run_len <<
                                                 // 4 | nbits] の下位2バイト
        "neg        a6, a6                  \n"  // a6 = nbits;

        // 符号化に必要な情報を32bitバッファに反映
        "bge        a13,a8, LABEL_CCPT_SKIP3\n"  // 32bitバッファが不足していれば出力
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP3: \n"
        "sub        a13,a13,a8              \n"  // bits_in -= len
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a7, a7                  \n"  // code <<= bits_in
        "add.n      a12,a12,a7              \n"  // bit_buffer += code

        "LABEL_CCPT_TEST: \n"
        // a6データを32bitバッファに反映
        "bge        a13,a6, LABEL_CCPT_SKIP4\n"
        "call0      LABEL_BIT_PUT_SUB       \n"
        "LABEL_CCPT_SKIP4: \n"
        "sub        a13,a13,a6              \n"  // bits_in -= nbits
        "ssl        a13                     \n"  // 左シフト量をSARレジスタにセット
        "sll        a5, a5                  \n"  // data <<= bits_in
        "add.n      a12,a12,a5              \n"  // bit_buffer += data

        "bgei       a11,8, LABEL_CCPT_ACLOOP2 \n"
        "j          LABEL_CCPT_ACLOOP       \n"  // ACループ再開

        // 32bitバッファから出力バッファに書込む
        // このサブルーチンは call0で呼び出すこと。
        // このサブルーチンは a9 を使用する。
        // a11:保存先バッファ残量
        // a12:32bitバッファ
        // a13:32bitバッファ残ビット数
        // a15:保存先アドレス
        "LABEL_BIT_PUT_SUB:                 \n"
        "movi.n     a9, 32                  \n"
        "sub        a9, a9, a13             \n"  // a9 = バッファ使用量を求める
                                                 // 32 - バッファ残量
        "srli       a9, a9, 3               \n"  // バッファ使用量>>3
                                                 // を取得(ループ回数)
        "addx8      a13,a9, a13             \n"  // 32bitバッファ残量回復
        "loop       a9, LABEL_BIT_PUT_LOOP  \n"  // ループ設定
        "srli       a9, a12,24              \n"  // a9 =
                                                 // 32bitバッファからデータ8bit取得
        "slli       a12,a12,8               \n"  // a12 =
                                                 // 32bitバッファを左8ビットシフト
        "s16i       a9, a15,0               \n"  // a9 を保存先に出力
                                                 // (意図的に2Byte出力し後続データを0にする。ESP32はアライメント境界を跨ぐ書込みが可能)
        "addi       a9, a9, 257             \n"  // a9+=257
        "srli       a9, a9, 8               \n"  // a9 >>= 8  得られる内容は
                                                 // (出力内容 == 255 ? 2 : 1)
        "add.n      a15,a15,a9              \n"  // 保存先アドレスを進める
        "sub        a11,a11,a9              \n"  // バッファ残量を減らす
        "LABEL_BIT_PUT_LOOP:                \n"
        // "addi.n     a9, a13,-25             \n"
        // "bltz       a9, LABEL_BIT_PUT_SUB   \n" //
        // バッファ残量が24以下なら再度出力
        "ret.n                              \n"

        //         "addi.n     a11,a11,-1              \n" //
        //         バッファ残量を1減らす "s8i        a9, a15,0               \n"
        //         // 0を保存先に出力
        // "LABEL_BIT_PUT_N255_0: \n"

        //         "extui      a9, a12,16, 8           \n" // a9 =
        //         データ8bit取得 "s8i        a9, a15,1               \n" // a9
        //         を保存先に出力 "addi       a9, a9, -255            \n" //
        //         次の行の分岐判定のために a9 から 255引く "bltz       a9,
        //         LABEL_BIT_PUT_N255_1\n" // if (a9<0) goto NOT255;
        //         出力内容が255だったかどうかで分岐 "addi.n     a15,a15,1 \n"
        //         // 保存先アドレスを1進める "addi.n     a11,a11,-1 \n" //
        //         バッファ残量を1減らす "s8i        a9, a15,1               \n"
        //         // 0を保存先に出力
        // "LABEL_BIT_PUT_N255_1: \n"

        //         "addi.n     a15,a15,2               \n" //
        //         保存先アドレスを2進める "addi.n     a13,a13,16 \n" // a13 =
        //         32bitバッファ残量を16回復 "slli       a12,a12,16 \n" // a12 =
        //         32bitバッファを左16ビットシフト "ret.n \n"
        // //*/

        // "LABEL_BIT_PUT_SUB2: \n"

        //         "movi.n     a9, 32                  \n"
        //         "sub        a9, a9, a13             \n" // a9 = 32 -
        //         バッファ残量 "srli       a9, a9, 3               \n" //
        //         バッファ残量>>3を取得 "addx8      a13,a9, a13             \n"
        //         // 32bitバッファ残量回復 "sub        a11,a11,a9 \n" //
        //         保存先バッファ残量を減らす "loop       a9, LABEL_BIT_PUT_LOOP
        //         \n" // ループ設定

        //         "srli       a9, a12,24              \n" // a9 =
        //         データ8bit取得 "slli       a12,a12,8               \n" // a12
        //         = 32bitバッファを左8ビットシフト "s16i       a9, a15,0 \n" //
        //         a9 を保存先に出力 "addi       a9, a9, -255            \n" //
        //         次の行の分岐判定のために a9 から 255引く "beqz       a9,
        //         LABEL_BIT_PUT_IS255 \n" // if (a9<0) goto NOT255;
        //         出力内容が255だったかどうかで分岐
        // "LABEL_BIT_PUT_NOT255:              \n"
        //         "addi.n     a15,a15,1               \n" // a15
        //         保存先アドレスを進める
        // "LABEL_BIT_PUT_LOOP:                \n"
        //         "ret.n                              \n"
        // "LABEL_BIT_PUT_IS255:               \n"
        //         "addi.n     a15,a15,1               \n" // a15
        //         保存先アドレスを進める "addi.n     a11,a11,-1 \n" // a11
        //         保存先バッファ残量を減らす
        //         // "s8i        a9, a15,0               \n" // 0を保存先に出力
        //         (JPEG仕様に合わせて0xFFの後に0x00を出力) "j
        //         LABEL_BIT_PUT_NOT255    \n"
    );
}

void jpeg_encoder::code_coefficients_pass_two(int component_num) {
    // uint out_buf_left = m_out_buf_left;
    // if (out_buf_left < 8) {
    //     flush_output_buffer();
    // }

    // debug
    //  GPIO.out1_w1ts.val = 1;
    bool cn_index = component_num;
    //*
    ccpt_asm_t s;
    s.fp_flush     = &jpeg_encoder::flush_output_buffer;
    s.jpe          = this;
    s.pOut_buf     = &m_pOut_buf;
    s.out_buf_left = &m_out_buf_left;
    s.bit_buffer   = m_bit_buffer;
    s.bits_in      = m_bits_in;
    s.codes_0      = m_huff_codes[0 + cn_index];  // codes_0;
    s.codes_1      = m_huff_codes[2 + cn_index];  // codes_1;
    // s.code_sizes_0 = m_huff_code_sizes[0 + cn_index]; // code_sizes_0;
    // s.code_sizes_1 = m_huff_code_sizes[2 + cn_index]; // code_sizes_1;
    // s.last_dc_val = &m_last_dc_val[component_num];

    code_coefficients_pass_two_asm(&s, m_coefficient_array,
                                   &m_last_dc_val[component_num]);
    // while (s.bits_in <= 16) {
    //     s.bits_in += 16;
    //     put_bits_asm_16(s.pOut_buf, s.out_buf_left, &s.bit_buffer);
    // }
    // m_pOut_buf = s.pOut_buf;
    // m_out_buf_left = s.out_buf_left;
    m_bits_in    = s.bits_in;
    m_bit_buffer = s.bit_buffer;
    return;
    //*/

    uint *codes_0 = m_huff_codes[0 + cn_index];
    uint *codes_1 = m_huff_codes[2 + cn_index];
    // uint8 *code_sizes_0 = m_huff_code_sizes[0 + cn_index];
    // uint8 *code_sizes_1 = m_huff_code_sizes[2 + cn_index];
    int16 *pSrc = m_coefficient_array;
    {
        int32 temp1                  = pSrc[0] - m_last_dc_val[component_num];
        m_last_dc_val[component_num] = pSrc[0];

        if (temp1) {
            uint32_t nbits = __builtin_clz(abs(temp1));
            if (temp1 < 0) {
                temp1 += ~0u >> nbits;
            }
            nbits = 32 - nbits;

            auto code_and_size = (uint16 *)&codes_0[nbits];
            uint32 len         = code_and_size[1];
            uint32 data        = code_and_size[0];
            // bits_in -= len;
            // bit_buffer += data << bits_in;
            // if (bits_in <= 16) {
            //     put_bits_asm_16(&pOut_buf, &out_buf_left, &bit_buffer);
            //     bits_in += 16;
            // }
            // bits_in -= nbits;
            // bit_buffer += temp1 << bits_in;
            put_bits(data, len);
            put_bits(temp1, nbits);
        } else {
            auto code_and_size = (uint16 *)&codes_0[0];
            uint32 len         = code_and_size[1];
            uint32 data        = code_and_size[0];
            // bits_in -= code_sizes_0[0];
            // bit_buffer += codes_0[0] << bits_in;
            put_bits(data, len);
            // put_bits(codes_0[0], code_sizes_0[0]);
        }
    }

    uint32 run_len = 0;
    int32 i        = 63;
    for (;;) {
        // if (bits_in <= 16) {
        //     bits_in += 16;
        //     put_bits_asm_16(&pOut_buf, &out_buf_left, &bit_buffer);
        // }

        run_len = 0;
        int32 temp1;
        while (i--) {
            temp1 = pSrc[1];
            ++pSrc;
            if (temp1) {
                break;
            }
            ++run_len;
        }  // while ((i >= 0) && temp1 == 0);
        // run_len -= i;
        if (i < 0) {
            break;
        }

        if (run_len >= 16) {
            auto code_and_size = (uint16 *)&codes_1[0xF0];
            uint32 size        = code_and_size[1];
            uint32 code        = code_and_size[0];
            do {
                // bits_in -= cs1;
                // bit_buffer += c1 << bits_in;
                // if (bits_in <= 16) {
                //     bits_in += 16;
                //     put_bits_asm_16(&pOut_buf, &out_buf_left, &bit_buffer);
                // }
                put_bits(code, size);
                run_len -= 16;
            } while (run_len >= 16);
        }
        //*
        uint32_t nbits = __builtin_clz(abs(temp1));
        if (temp1 < 0) {
            temp1 += ~0u >> nbits;
        }
        nbits = 32 - nbits;

        ///////// アセンブラ版
        /*/
                    uint32 nbits;
                    __asm__ __volatile__(
                        "abs    %0, %1      \n"
                        "nsau   %0, %0      \n"
                        "neg    %0, %0      \n"
                        "addi.n %0, %0, 32  \n"
                        : "=r" (nbits) : "r" (temp1)
                    );
        //*/
        //////////

        uint32 idx         = (run_len << 4) + nbits;
        auto code_and_size = (uint16 *)&codes_1[idx];
        uint32 data        = code_and_size[0];
        uint32 len         = code_and_size[1];
        // bits_in -= len;
        // bit_buffer += data << bits_in;
        // if (bits_in <= 16) {
        //     bits_in += 16;
        //     put_bits_asm_16(&pOut_buf, &out_buf_left, &bit_buffer);
        // }
        temp1 += data << nbits;
        nbits += len;
        if (nbits > 24) {
            put_bits(temp1 >> 16, nbits - 16);
            temp1 &= 0xFFFF;
            nbits = 16;
        }
        put_bits(temp1, nbits);

        // bits_in -= nbits;
        // bit_buffer += temp1 << bits_in;
    }
    if (run_len) {
        auto code_and_size = (uint16 *)&codes_1[0];
        uint32 len         = code_and_size[1];
        uint32 data        = code_and_size[0];
        put_bits(data, len);
        // put_bits(codes_1[0], code_sizes_1[0]);
        // bits_in -= code_sizes_1[0];
        // bit_buffer += codes_1[0] << bits_in;

        // if (bits_in <= 16) {
        //     bits_in += 16;
        //     put_bits_asm_16(&pOut_buf, &out_buf_left, &bit_buffer);
        // }
    }

    // debug
    //  GPIO.out1_w1tc.val = 1;
}

void jpeg_encoder::code_block(int component_num) {
#ifdef _SOC_GPIO_STRUCT_H_
    GPIO.out1_w1ts.val = 3;
#endif
    // DCT2D(m_sample_array);
    dct2d_asm(m_sample_array, dct2d_tbl);
#ifdef _SOC_GPIO_STRUCT_H_
    GPIO.out1_w1tc.val = 1;
#endif
    // load_quantized_coefficients(component_num);
    load_quantized_coefficients_asm(m_coefficient_array, m_sample_array,
                                    m_quantization32_tables[component_num > 0],
                                    s_zag);
#ifdef _SOC_GPIO_STRUCT_H_
    GPIO.out1_w1ts.val = 1;
#endif
    code_coefficients_pass_two(component_num);
#ifdef _SOC_GPIO_STRUCT_H_
    GPIO.out1_w1tc.val = 3;
#endif
}

void jpeg_encoder::process_mcu_row() {
    m_mcu_y_ofs = 0;
    if (m_num_components == 1) {
        for (int i = 0; i < m_mcus_per_row; i++) {
            load_block_8_8_grey(i);
            code_block(0);
        }
    } else if ((m_comp_h_samp[0] == 1) && (m_comp_v_samp[0] == 1)) {
        for (int i = 0; i < m_mcus_per_row; i++) {
            load_block_8_8(i, 0, 0);
            code_block(0);
            load_block_8_8(i, 0, 1);
            code_block(1);
            load_block_8_8(i, 0, 2);
            code_block(2);
        }
    } else if ((m_comp_h_samp[0] == 2) && (m_comp_v_samp[0] == 1)) {
        for (int i = 0; i < m_mcus_per_row; i++) {
            load_block_8_8(i * 2 + 0, 0, 0);
            code_block(0);
            load_block_8_8(i * 2 + 1, 0, 0);
            code_block(0);
            load_block_16_8_8(i, 1);
            code_block(1);
            load_block_16_8_8(i, 2);
            code_block(2);
        }
    } else if ((m_comp_h_samp[0] == 2) && (m_comp_v_samp[0] == 2)) {
        for (int i = 0; i < m_mcus_per_row; i++) {
            load_block_8_8(i * 2 + 0, 0, 0);
            code_block(0);
            load_block_8_8(i * 2 + 1, 0, 0);
            code_block(0);
            load_block_8_8(i * 2 + 0, 1, 0);
            code_block(0);
            load_block_8_8(i * 2 + 1, 1, 0);
            code_block(0);
            load_block_16_8(i, 1);
            code_block(1);
            load_block_16_8(i, 2);
            code_block(2);
        }
    }
}

void jpeg_encoder::load_mcu(const void *pSrc) {
    // const uint8* Psrc = reinterpret_cast<const uint8*>(pSrc);

    // uint8* pDst = m_mcu_lines[m_mcu_y_ofs]; // OK to write up to
    // m_image_bpl_xlt bytes to pDst

    // if (m_num_components == 1) {
    //     if (m_image_bpp == 3)
    //         RGB_to_Y(pDst, Psrc, m_image_x);
    //     else
    //         memcpy(pDst, Psrc, m_image_x);
    // } else {
    //     if (m_image_bpp == 3)
    //         RGB_to_YCC(pDst, Psrc, m_image_x);
    //     else
    //         Y_to_YCC(pDst, Psrc, m_image_x);
    // }

    // // Possibly duplicate pixels at end of scanline if not a multiple of 8 or
    // 16 if (m_num_components == 1)
    //     memset(m_mcu_lines[m_mcu_y_ofs] + m_image_bpl_xlt,
    //     pDst[m_image_bpl_xlt - 1], m_image_x_mcu - m_image_x);
    // else
    // {
    //     const uint8 y = pDst[m_image_bpl_xlt - 3 + 0], cb =
    //     pDst[m_image_bpl_xlt - 3 + 1], cr = pDst[m_image_bpl_xlt - 3 + 2];
    //     uint8 *q = m_mcu_lines[m_mcu_y_ofs] + m_image_bpl_xlt;
    //     for (int i = m_image_x; i < m_image_x_mcu; i++)
    //     {
    //         *q++ = y; *q++ = cb; *q++ = cr;
    //     }
    // }

    // if (++m_mcu_y_ofs == m_mcu_y)
    // {
    //     process_mcu_row();
    //     m_mcu_y_ofs = 0;
    // }
}

void jpeg_encoder::load_mcu565(const void *pSrc) {
    const uint8 *Psrc = reinterpret_cast<const uint8 *>(pSrc);

    int16 *pDst = m_mcu_lines[m_mcu_y_ofs];  // OK to write up to
                                             // m_image_bpl_xlt bytes to pDst

    // if (m_num_components == 1) {
    //     if (m_image_bpp == 3)
    //         RGB_to_Y(pDst, Psrc, m_image_x);
    //     else
    //         memcpy(pDst, Psrc, m_image_x);
    // } else {
    //     if (m_image_bpp == 3) {
    // ASM版
    RGB565_to_YCC_asm_t asm_data;
    RGB565_to_YCC_asm(&asm_data, pDst, (const uint16_t *)Psrc, m_image_x);
    // C++版
    // RGB565_to_YCC(pDst, (const uint16_t*)Psrc, m_image_x);
    //     } else {
    //         Y_to_YCC(pDst, Psrc, m_image_x);
    //     }
    // }

    // Possibly duplicate pixels at end of scanline if not a multiple of 8 or 16
    // if (m_num_components == 1)
    //     memset(m_mcu_lines[m_mcu_y_ofs] + m_image_bpl_xlt,
    //     pDst[m_image_bpl_xlt - 1], m_image_x_mcu - m_image_x);
    // else
    // {
    const int16 y  = pDst[m_image_bpl_xlt - 3 + 0],
                cb = pDst[m_image_bpl_xlt - 3 + 1],
                cr = pDst[m_image_bpl_xlt - 3 + 2];
    int16 *q       = m_mcu_lines[m_mcu_y_ofs] + m_image_bpl_xlt;
    for (int i = m_image_x; i < m_image_x_mcu; i++) {
        *q++ = y;
        *q++ = cb;
        *q++ = cr;
    }
    // }
}

// Quantization table generation.
void jpeg_encoder::compute_quant_table(uint8 *pDst, int32 *pDst32,
                                       const uint8 *pSrc) {
    int32 q;
    if (m_params.m_quality < 50)
        q = 5000 / m_params.m_quality;
    else
        q = 200 - m_params.m_quality * 2;
    for (int i = 0; i < 64; i++) {
        int32 j   = *pSrc++;
        j         = (j * q + 50L) / 100L;
        j         = JPGE_MIN(JPGE_MAX(j, 1), 255);
        *pDst++   = j;
        *pDst32++ = j << 18;
    }
}

// Higher-level methods.
bool jpeg_encoder::jpg_open(int p_x_res, int p_y_res, int src_channels) {
    m_num_components = 3;
    switch (m_params.m_subsampling) {
        case Y_ONLY: {
            m_num_components = 1;
            m_comp_h_samp[0] = 1;
            m_comp_v_samp[0] = 1;
            m_mcu_x          = 8;
            m_mcu_y          = 8;
            break;
        }
        case H1V1: {
            m_comp_h_samp[0] = 1;
            m_comp_v_samp[0] = 1;
            m_comp_h_samp[1] = 1;
            m_comp_v_samp[1] = 1;
            m_comp_h_samp[2] = 1;
            m_comp_v_samp[2] = 1;
            m_mcu_x          = 8;
            m_mcu_y          = 8;
            break;
        }
        case H2V1: {
            m_comp_h_samp[0] = 2;
            m_comp_v_samp[0] = 1;
            m_comp_h_samp[1] = 1;
            m_comp_v_samp[1] = 1;
            m_comp_h_samp[2] = 1;
            m_comp_v_samp[2] = 1;
            m_mcu_x          = 16;
            m_mcu_y          = 8;
            break;
        }
        case H2V2: {
            m_comp_h_samp[0] = 2;
            m_comp_v_samp[0] = 2;
            m_comp_h_samp[1] = 1;
            m_comp_v_samp[1] = 1;
            m_comp_h_samp[2] = 1;
            m_comp_v_samp[2] = 1;
            m_mcu_x          = 16;
            m_mcu_y          = 16;
        }
    }

    m_image_x       = p_x_res;
    m_image_y       = p_y_res;
    m_image_bpp     = src_channels;
    m_image_bpl     = m_image_x * src_channels;
    m_image_x_mcu   = (m_image_x + m_mcu_x - 1) & (~(m_mcu_x - 1));
    m_image_y_mcu   = (m_image_y + m_mcu_y - 1) & (~(m_mcu_y - 1));
    m_image_bpl_xlt = m_image_x * m_num_components;
    m_image_bpl_mcu = m_image_x_mcu * m_num_components;
    m_mcus_per_row  = m_image_x_mcu / m_mcu_x;
    m_out_buf_index = 0;

    if ((m_mcu_lines[0] = static_cast<int16 *>(
             jpge_malloc(sizeof(int16) * m_image_bpl_mcu * m_mcu_y))) == NULL) {
        return false;
    }
    for (int i = 1; i < m_mcu_y; i++)
        m_mcu_lines[i] = m_mcu_lines[i - 1] + m_image_bpl_mcu;

    if (!m_huff_initialized) {
        m_huff_initialized = true;

        memcpy(m_huff_bits[0 + 0], s_dc_lum_bits, 17);
        memcpy(m_huff_val[0 + 0], s_dc_lum_val, DC_LUM_CODES);
        memcpy(m_huff_bits[2 + 0], s_ac_lum_bits, 17);
        memcpy(m_huff_val[2 + 0], s_ac_lum_val, AC_LUM_CODES);
        memcpy(m_huff_bits[0 + 1], s_dc_chroma_bits, 17);
        memcpy(m_huff_val[0 + 1], s_dc_chroma_val, DC_CHROMA_CODES);
        memcpy(m_huff_bits[2 + 1], s_ac_chroma_bits, 17);
        memcpy(m_huff_val[2 + 1], s_ac_chroma_val, AC_CHROMA_CODES);

        compute_huffman_table(&m_huff_codes[0 + 0][0],
                              &m_huff_code_sizes[0 + 0][0], m_huff_bits[0 + 0],
                              m_huff_val[0 + 0]);
        compute_huffman_table(&m_huff_codes[2 + 0][0],
                              &m_huff_code_sizes[2 + 0][0], m_huff_bits[2 + 0],
                              m_huff_val[2 + 0]);
        compute_huffman_table(&m_huff_codes[0 + 1][0],
                              &m_huff_code_sizes[0 + 1][0], m_huff_bits[0 + 1],
                              m_huff_val[0 + 1]);
        compute_huffman_table(&m_huff_codes[2 + 1][0],
                              &m_huff_code_sizes[2 + 1][0], m_huff_bits[2 + 1],
                              m_huff_val[2 + 1]);
    }

    return reinit(m_params.m_quality);
}

bool jpeg_encoder::reinit(int quality) {
    if (!m_mcu_lines[0]) {
        return false;
    }

    m_params.m_quality = quality;
    if (m_last_quality != m_params.m_quality) {
        m_last_quality = m_params.m_quality;
        compute_quant_table(m_quantization8_tables[0],
                            m_quantization32_tables[0], s_std_lum_quant);
        compute_quant_table(m_quantization8_tables[1],
                            m_quantization32_tables[1], s_std_croma_quant);
    }

    m_out_buf_left = JPGE_OUT_BUF_SIZE;
    m_pOut_buf     = m_out_buf_array[m_out_buf_index];
    m_bit_buffer   = 0;
    m_bits_in      = 32;
    m_mcu_y_ofs    = 0;
    m_pass_num     = 2;
    memset(m_last_dc_val, 0, 3 * sizeof(m_last_dc_val[0]));

    // Emit all markers at beginning of image file.
    emit_marker(M_SOI);
    emit_jfif_app0();
    emit_dqt();
    emit_sof();
    emit_dhts();
    emit_sos();

    return m_all_stream_writes_succeeded;
}

bool jpeg_encoder::process_end_of_image() {
    if (m_mcu_y_ofs) {
        if (m_mcu_y_ofs < 16) {  // check here just to shut up static analysis
            for (int i = m_mcu_y_ofs; i < m_mcu_y; i++) {
                memcpy(m_mcu_lines[i], m_mcu_lines[m_mcu_y_ofs - 1],
                       sizeof(int16) * m_image_bpl_mcu);
            }
        }
        process_mcu_row();
    }

    put_bits(0x7F, 7);
    emit_marker(M_EOI);
    flush_output_buffer();
    m_all_stream_writes_succeeded =
        m_all_stream_writes_succeeded && m_pStream->put_buf(NULL, 0);
    m_pass_num++;  // purposely bump up m_pass_num, for debugging
    return true;
}

void jpeg_encoder::clear() {
    m_mcu_lines[0]                = NULL;
    m_pass_num                    = 0;
    m_all_stream_writes_succeeded = true;
}

jpeg_encoder::jpeg_encoder() {
    clear();
}

jpeg_encoder::~jpeg_encoder() {
    deinit();
}

bool jpeg_encoder::init(output_stream *pStream, int width, int height,
                        int src_channels, const params &comp_params) {
    deinit();
    if (((!pStream) || (width < 1) || (height < 1)) ||
        ((src_channels != 1) && (src_channels != 3) && (src_channels != 4)) ||
        (!comp_params.check()))
        return false;
    m_pStream = pStream;
    m_params  = comp_params;
    return jpg_open(width, height, src_channels);
}

void jpeg_encoder::deinit() {
    jpge_free(m_mcu_lines[0]);
    clear();
}

bool jpeg_encoder::process_scanline565(const void *pScanline) {
    if ((m_pass_num < 1) || (m_pass_num > 2)) {
        return false;
    }
    if (m_all_stream_writes_succeeded) {
        if (!pScanline) {
            if (!process_end_of_image()) {
                return false;
            }
        } else {
            load_mcu565(pScanline);

            if (++m_mcu_y_ofs == m_mcu_y) {
                m_mcu_y_ofs = 0;
                // process_mcu_row();
            }
        }
    }
    return m_all_stream_writes_succeeded;
}

bool jpeg_encoder::process_scanline(const void *pScanline) {
    if ((m_pass_num < 1) || (m_pass_num > 2)) {
        return false;
    }
    if (m_all_stream_writes_succeeded) {
        if (!pScanline) {
            if (!process_end_of_image()) {
                return false;
            }
        } else {
            load_mcu(pScanline);
        }
    }
    return m_all_stream_writes_succeeded;
}

}  // namespace jpge
