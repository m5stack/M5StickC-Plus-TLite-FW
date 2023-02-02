
#include "mlx90640.hpp"

#include "i2c_master.hpp"

#include <esp_log.h>
#include <algorithm>
#include <cstring>
#include <cmath>

// 不良ピクセルのデバッグのために疑似的に設定した不良ピクセルの番号;
// #define DEBUG_BROKENPIXEL 100

namespace m5 {
static constexpr const uint8_t noise_tbl[] = {
    0,  0,  0,  1,  2,  5,  8,  13, 20, 28, 39,  52,  67,  86,  107, 132, 160,
    0,  0,  0,  1,  3,  5,  9,  14, 20, 29, 39,  52,  68,  86,  108, 132, 160,
    0,  0,  1,  2,  3,  6,  9,  14, 21, 30, 41,  54,  69,  88,  109, 134, 162,
    1,  1,  1,  2,  4,  7,  11, 16, 23, 32, 42,  56,  72,  90,  112, 137, 165,
    1,  2,  2,  3,  5,  8,  12, 18, 25, 34, 45,  59,  75,  94,  116, 141, 170,
    3,  3,  4,  5,  7,  10, 15, 21, 28, 37, 49,  63,  79,  98,  121, 146, 175,
    4,  5,  6,  7,  10, 13, 18, 24, 32, 42, 54,  68,  85,  104, 127, 153, 182,
    7,  7,  8,  10, 13, 17, 22, 28, 37, 47, 59,  74,  91,  111, 134, 161, 191,
    11, 11, 12, 14, 17, 21, 27, 34, 42, 53, 66,  81,  99,  119, 143, 170, 200,
    15, 15, 17, 19, 22, 27, 33, 40, 49, 60, 74,  89,  108, 129, 153, 181, 212,
    21, 21, 22, 25, 29, 33, 40, 48, 57, 69, 83,  99,  118, 140, 165, 193, 225,
    27, 28, 29, 32, 36, 41, 48, 56, 67, 79, 93,  110, 130, 152, 178, 207, 239,
    35, 36, 38, 41, 45, 51, 58, 67, 77, 90, 105, 123, 143, 166, 193, 222, 255};

inline uint16_t bswap16(uint16_t data) {
    return (data << 8) + (data >> 8);
}

static constexpr float SCALEALPHA = 0.000001;
static constexpr size_t TA_SHIFT = 8;  // Default shift for MLX90640 in open air
// static constexpr size_t COLS = 32;
// static constexpr size_t ROWS = 24;

struct MLX90640_params_t {
    int16_t kVdd;
    int16_t vdd25;
    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;
    int16_t gainEE;
    float tgc;
    float cpKv;
    float cpKta;
    uint8_t resolutionEE;
    uint8_t calibrationModeEE;
    float KsTa;
    float ksTo[5];
    int16_t ct[5];
    uint16_t alpha[768];
    uint8_t alphaScale;
    int16_t offset[768];
    int8_t kta[768];
    uint8_t ktaScale;
    int8_t kv[768];
    uint8_t kvScale;
    float cpAlpha[2];
    int16_t cpOffset[2];
    float ilChessC[3];
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];

    static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2) {
        int pixPosDif = pix1 - pix2;
        if (pixPosDif > -34 && pixPosDif < -30) {
            return -6;
        }
        if (pixPosDif > -2 && pixPosDif < 2) {
            return -6;
        }
        if (pixPosDif > 30 && pixPosDif < 34) {
            return -6;
        }
        return 0;
    }

    void setVDDParameters(const uint16_t *eeData) {
        int tmp = eeData[0x33] >> 8;  // vdd  reg 0x2433
        if (tmp > 127) {
            tmp -= 256;
        }
        this->kVdd  = tmp << 5;
        this->vdd25 = (((eeData[0x33] & 0xFF) - 256) << 5) - 8192;
    }

    void setPTATParameters(const uint16_t *eeData) {
        int tmp = eeData[0x32] >> 10;  // PTA  reg 0x2432
        if (tmp > 31) {
            tmp -= 64;
        }
        this->KvPTAT = (float)tmp / 4096.0f;

        tmp = eeData[0x32] & 0x03FF;
        if (tmp > 511) {
            tmp -= 1024;
        }
        this->KtPTAT = (float)tmp / 8.0f;

        this->vPTAT25   = eeData[0x31];                            // reg 0x2431
        this->alphaPTAT = (float)(eeData[0x10] >> 12) / 4 + 8.0f;  // reg 0x2410
    }

    void setGainParameters(const uint16_t *eeData) {
        int tmp = eeData[0x30];  // reg 0x2430
        if (tmp > 32767) {
            tmp -= 65536;
        }
        this->gainEE = tmp;
    }

    void setTgcParameters(const uint16_t *eeData) {
        int tmp = eeData[0x3C] & 0xFF;  // reg 0x243C
        if (tmp > 127) {
            tmp -= 256;
        }
        this->tgc = (float)tmp / 32.0f;
    }

    void setResolutionParameters(const uint16_t *eeData) {
        this->resolutionEE = (eeData[0x38] >> 12) & 0x3;  // reg 0x2438
    }

    void setKsTaParameters(const uint16_t *eeData) {
        int tmp = eeData[0x3C] >> 8;  // reg 0x243C
        if (tmp > 127) {
            tmp -= 256;
        }
        this->KsTa = (float)tmp / 8192.0f;
    }

    void setKsToParameters(const uint16_t *eeData) {
        this->ksTo[0] = eeData[0x3D] & 0xFF;  // reg 0x243D
        this->ksTo[1] = eeData[0x3D] >> 8;
        this->ksTo[2] = eeData[0x3E] & 0xFF;  // reg 0x243E
        this->ksTo[3] = eeData[0x3E] >> 8;
        this->ksTo[4] = -0.0002;

        int step = ((eeData[0x3F] >> 12) & 0x3) * 10;

        this->ct[0] = -40;
        this->ct[1] = 0;
        this->ct[2] = ((eeData[0x3F] >> 4) & 0x0F) * step;
        this->ct[3] = ((eeData[0x3F] >> 8) & 0x0F) * step + this->ct[2];
        this->ct[4] = 400;

        int KsToScale = 1 << ((eeData[0x3F] & 0x0F) + 8);

        for (int i = 0; i < 4; i++) {
            float tmp = this->ksTo[i];
            if (tmp > 127) {
                tmp -= 256;
            }
            this->ksTo[i] = tmp / KsToScale;
        }
    }

    void setCPParameters(const uint16_t *eeData) {
        float alphaSP[2];
        int16_t offsetSP[2];
        float cpKv;
        float cpKta;
        uint8_t alphaScale;
        uint8_t ktaScale1;
        uint8_t kvScale;

        alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;

        offsetSP[0] = (eeData[58] & 0x03FF);
        if (offsetSP[0] > 511) {
            offsetSP[0] = offsetSP[0] - 1024;
        }

        offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
        if (offsetSP[1] > 31) {
            offsetSP[1] = offsetSP[1] - 64;
        }
        offsetSP[1] = offsetSP[1] + offsetSP[0];

        alphaSP[0] = (eeData[57] & 0x03FF);
        if (alphaSP[0] > 511) {
            alphaSP[0] = alphaSP[0] - 1024;
        }
        alphaSP[0] = alphaSP[0] / pow(2, (double)alphaScale);

        alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
        if (alphaSP[1] > 31) {
            alphaSP[1] = alphaSP[1] - 64;
        }
        alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];

        cpKta = (eeData[59] & 0x00FF);
        if (cpKta > 127) {
            cpKta = cpKta - 256;
        }
        ktaScale1   = ((eeData[56] & 0x00F0) >> 4) + 8;
        this->cpKta = cpKta / pow(2, (double)ktaScale1);

        cpKv = (eeData[59] & 0xFF00) >> 8;
        if (cpKv > 127) {
            cpKv = cpKv - 256;
        }
        kvScale    = (eeData[56] & 0x0F00) >> 8;
        this->cpKv = cpKv / pow(2, (double)kvScale);

        this->cpAlpha[0]  = alphaSP[0];
        this->cpAlpha[1]  = alphaSP[1];
        this->cpOffset[0] = offsetSP[0];
        this->cpOffset[1] = offsetSP[1];
    }

    void setAlphaParameters(const uint16_t *eeData) {
        int accRow[24];
        int accColumn[32];
        int p = 0;
        int alphaRef;
        uint8_t alphaScale;
        uint8_t accRowScale;
        uint8_t accColumnScale;
        uint8_t accRemScale;
        float alphaTemp[768];
        float temp;

        accRemScale    = eeData[32] & 0x000F;
        accColumnScale = (eeData[32] & 0x00F0) >> 4;
        accRowScale    = (eeData[32] & 0x0F00) >> 8;
        alphaScale     = ((eeData[32] & 0xF000) >> 12) + 30;
        alphaRef       = eeData[33];

        for (int i = 0; i < 6; i++) {
            p             = i * 4;
            accRow[p + 0] = (eeData[34 + i] & 0x000F);
            accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
            accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
            accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < 24; i++) {
            if (accRow[i] > 7) {
                accRow[i] = accRow[i] - 16;
            }
        }

        for (int i = 0; i < 8; i++) {
            p                = i * 4;
            accColumn[p + 0] = (eeData[40 + i] & 0x000F);
            accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
            accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
            accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < 32; i++) {
            if (accColumn[i] > 7) {
                accColumn[i] = accColumn[i] - 16;
            }
        }

        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 32; j++) {
                p            = 32 * i + j;
                alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
                if (alphaTemp[p] > 31) {
                    alphaTemp[p] = alphaTemp[p] - 64;
                }
                alphaTemp[p] = alphaTemp[p] * (1 << accRemScale);
                alphaTemp[p] =
                    (alphaRef + (accRow[i] << accRowScale) +
                     (accColumn[j] << accColumnScale) + alphaTemp[p]);
                alphaTemp[p] = alphaTemp[p] / pow(2, (double)alphaScale);
                alphaTemp[p] =
                    alphaTemp[p] -
                    this->tgc * (this->cpAlpha[0] + this->cpAlpha[1]) / 2;
                alphaTemp[p] = SCALEALPHA / alphaTemp[p];
            }
        }

        temp = alphaTemp[0];
        for (int i = 1; i < 768; i++) {
            if (alphaTemp[i] > temp) {
                temp = alphaTemp[i];
            }
        }

        alphaScale = 0;
        while (temp < 32768) {
            temp       = temp * 2;
            alphaScale = alphaScale + 1;
        }

        for (int i = 0; i < 768; i++) {
            temp           = alphaTemp[i] * pow(2, (double)alphaScale);
            this->alpha[i] = (temp + 0.5);
        }
        this->alphaScale = alphaScale;
    }

    void setOffsetParameters(const uint16_t *eeData) {
        int occRow[24];
        int occColumn[32];
        int p = 0;
        int16_t offsetRef;
        uint8_t occRowScale;
        uint8_t occColumnScale;
        uint8_t occRemScale;

        occRemScale    = (eeData[16] & 0x000F);
        occColumnScale = (eeData[16] & 0x00F0) >> 4;
        occRowScale    = (eeData[16] & 0x0F00) >> 8;
        offsetRef      = eeData[17];
        if (offsetRef > 32767) {
            offsetRef = offsetRef - 65536;
        }

        for (int i = 0; i < 6; i++) {
            p             = i * 4;
            occRow[p + 0] = (eeData[18 + i] & 0x000F);
            occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
            occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
            occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < 24; i++) {
            if (occRow[i] > 7) {
                occRow[i] = occRow[i] - 16;
            }
        }

        for (int i = 0; i < 8; i++) {
            p                = i * 4;
            occColumn[p + 0] = (eeData[24 + i] & 0x000F);
            occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
            occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
            occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < 32; i++) {
            if (occColumn[i] > 7) {
                occColumn[i] = occColumn[i] - 16;
            }
        }

        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 32; j++) {
                p               = 32 * i + j;
                this->offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
                if (this->offset[p] > 31) {
                    this->offset[p] = this->offset[p] - 64;
                }
                this->offset[p] = this->offset[p] * (1 << occRemScale);
                this->offset[p] =
                    (offsetRef + (occRow[i] << occRowScale) +
                     (occColumn[j] << occColumnScale) + this->offset[p]);
            }
        }
    }

    void setKtaPixelParameters(const uint16_t *eeData) {
        int p = 0;
        int8_t KtaRC[4];
        int8_t KtaRoCo;
        int8_t KtaRoCe;
        int8_t KtaReCo;
        int8_t KtaReCe;
        uint8_t ktaScale1;
        uint8_t ktaScale2;
        uint8_t split;
        float ktaTemp[768];
        float temp;

        KtaRoCo = (eeData[54] & 0xFF00) >> 8;
        if (KtaRoCo > 127) {
            KtaRoCo = KtaRoCo - 256;
        }
        KtaRC[0] = KtaRoCo;

        KtaReCo = (eeData[54] & 0x00FF);
        if (KtaReCo > 127) {
            KtaReCo = KtaReCo - 256;
        }
        KtaRC[2] = KtaReCo;

        KtaRoCe = (eeData[55] & 0xFF00) >> 8;
        if (KtaRoCe > 127) {
            KtaRoCe = KtaRoCe - 256;
        }
        KtaRC[1] = KtaRoCe;

        KtaReCe = (eeData[55] & 0x00FF);
        if (KtaReCe > 127) {
            KtaReCe = KtaReCe - 256;
        }
        KtaRC[3] = KtaReCe;

        ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
        ktaScale2 = (eeData[56] & 0x000F);

        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 32; j++) {
                p          = 32 * i + j;
                split      = 2 * (p / 32 - (p / 64) * 2) + p % 2;
                ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
                if (ktaTemp[p] > 3) {
                    ktaTemp[p] = ktaTemp[p] - 8;
                }
                ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
                ktaTemp[p] = KtaRC[split] + ktaTemp[p];
                ktaTemp[p] = ktaTemp[p] / pow(2, (double)ktaScale1);
                // ktaTemp[p] = ktaTemp[p] * this->offset[p];
            }
        }

        temp = fabs(ktaTemp[0]);
        for (int i = 1; i < 768; i++) {
            if (fabs(ktaTemp[i]) > temp) {
                temp = fabs(ktaTemp[i]);
            }
        }

        ktaScale1 = 0;
        while (temp < 64) {
            temp      = temp * 2;
            ktaScale1 = ktaScale1 + 1;
        }

        for (int i = 0; i < 768; i++) {
            temp = ktaTemp[i] * pow(2, (double)ktaScale1);
            if (temp < 0) {
                this->kta[i] = (temp - 0.5);
            } else {
                this->kta[i] = (temp + 0.5);
            }
        }

        this->ktaScale = ktaScale1;
    }

    void setKvPixelParameters(const uint16_t *eeData) {
        int p = 0;
        int8_t KvT[4];
        int8_t KvRoCo;
        int8_t KvRoCe;
        int8_t KvReCo;
        int8_t KvReCe;
        uint8_t kvScale;
        uint8_t split;
        float kvTemp[768];
        float temp;

        KvRoCo = (eeData[52] & 0xF000) >> 12;
        if (KvRoCo > 7) {
            KvRoCo = KvRoCo - 16;
        }
        KvT[0] = KvRoCo;

        KvReCo = (eeData[52] & 0x0F00) >> 8;
        if (KvReCo > 7) {
            KvReCo = KvReCo - 16;
        }
        KvT[2] = KvReCo;

        KvRoCe = (eeData[52] & 0x00F0) >> 4;
        if (KvRoCe > 7) {
            KvRoCe = KvRoCe - 16;
        }
        KvT[1] = KvRoCe;

        KvReCe = (eeData[52] & 0x000F);
        if (KvReCe > 7) {
            KvReCe = KvReCe - 16;
        }
        KvT[3] = KvReCe;

        kvScale = (eeData[56] & 0x0F00) >> 8;

        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 32; j++) {
                p         = 32 * i + j;
                split     = 2 * (p / 32 - (p / 64) * 2) + p % 2;
                kvTemp[p] = KvT[split];
                kvTemp[p] = kvTemp[p] / pow(2, (double)kvScale);
                // kvTemp[p] = kvTemp[p] * this->offset[p];
            }
        }

        temp = fabs(kvTemp[0]);
        for (int i = 1; i < 768; i++) {
            if (fabs(kvTemp[i]) > temp) {
                temp = fabs(kvTemp[i]);
            }
        }

        kvScale = 0;
        while (temp < 64) {
            temp    = temp * 2;
            kvScale = kvScale + 1;
        }

        for (int i = 0; i < 768; i++) {
            temp = kvTemp[i] * pow(2, (double)kvScale);
            if (temp < 0) {
                this->kv[i] = (temp - 0.5);
            } else {
                this->kv[i] = (temp + 0.5);
            }
        }

        this->kvScale = kvScale;
    }

    void setCILCParameters(const uint16_t *eeData) {
        float ilChessC[3];
        uint8_t calibrationModeEE;

        calibrationModeEE = (eeData[10] & 0x0800) >> 4;
        calibrationModeEE = calibrationModeEE ^ 0x80;

        ilChessC[0] = (eeData[53] & 0x003F);
        if (ilChessC[0] > 31) {
            ilChessC[0] = ilChessC[0] - 64;
        }
        ilChessC[0] = ilChessC[0] / 16.0f;

        ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
        if (ilChessC[1] > 15) {
            ilChessC[1] = ilChessC[1] - 32;
        }
        ilChessC[1] = ilChessC[1] / 2.0f;

        ilChessC[2] = (eeData[53] & 0xF800) >> 11;
        if (ilChessC[2] > 15) {
            ilChessC[2] = ilChessC[2] - 32;
        }
        ilChessC[2] = ilChessC[2] / 8.0f;

        this->calibrationModeEE = calibrationModeEE;
        this->ilChessC[0]       = ilChessC[0];
        this->ilChessC[1]       = ilChessC[1];
        this->ilChessC[2]       = ilChessC[2];
    }

    int setDeviatingPixels(const uint16_t *eeData) {
        uint16_t pixCnt        = 0;
        uint16_t brokenPixCnt  = 0;
        uint16_t outlierPixCnt = 0;
        int warn               = 0;
        int i;

        for (pixCnt = 0; pixCnt < 5; pixCnt++) {
            this->brokenPixels[pixCnt]  = 0xFFFF;
            this->outlierPixels[pixCnt] = 0xFFFF;
        }

        pixCnt = 0;
        while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5) {
            if (eeData[pixCnt + 64] == 0) {
                this->brokenPixels[brokenPixCnt] = pixCnt;
                brokenPixCnt                     = brokenPixCnt + 1;
            } else if ((eeData[pixCnt + 64] & 0x0001) != 0) {
                this->outlierPixels[outlierPixCnt] = pixCnt;
                outlierPixCnt                      = outlierPixCnt + 1;
            }

            pixCnt = pixCnt + 1;
        }
#if defined(DEBUG_BROKENPIXEL)
        this->brokenPixels[brokenPixCnt] = DEBUG_BROKENPIXEL;
        brokenPixCnt                     = brokenPixCnt + 1;
#endif

        if (brokenPixCnt > 4) {
            warn = -3;
        } else if (outlierPixCnt > 4) {
            warn = -4;
        } else if ((brokenPixCnt + outlierPixCnt) > 4) {
            warn = -5;
        } else {
            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
                for (i = pixCnt + 1; i < brokenPixCnt; i++) {
                    warn = CheckAdjacentPixels(this->brokenPixels[pixCnt],
                                               this->brokenPixels[i]);
                    if (warn != 0) {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++) {
                for (i = pixCnt + 1; i < outlierPixCnt; i++) {
                    warn = CheckAdjacentPixels(this->outlierPixels[pixCnt],
                                               this->outlierPixels[i]);
                    if (warn != 0) {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
                for (i = 0; i < outlierPixCnt; i++) {
                    warn = CheckAdjacentPixels(this->brokenPixels[pixCnt],
                                               this->outlierPixels[i]);
                    if (warn != 0) {
                        return warn;
                    }
                }
            }
        }
        return warn;
    }

    void setParam(const uint16_t *eeData) {
        setVDDParameters(eeData);
        setPTATParameters(eeData);
        setGainParameters(eeData);
        setTgcParameters(eeData);
        setResolutionParameters(eeData);
        setKsTaParameters(eeData);
        setKsToParameters(eeData);
        setCPParameters(eeData);
        setAlphaParameters(eeData);
        setOffsetParameters(eeData);
        setKtaPixelParameters(eeData);
        setKvPixelParameters(eeData);
        setCILCParameters(eeData);
        setDeviatingPixels(eeData);
    }

    float MLX90640_GetVdd(const uint16_t *frameData) const {
        float vdd;
        float resolutionCorrection;

        int resolutionRAM;

        vdd = frameData[810];
        if (vdd > 32767) {
            vdd = vdd - 65536;
        }
        resolutionRAM = (frameData[832] & 0x0C00) >> 10;
        resolutionCorrection =
            pow(2, (double)this->resolutionEE) / pow(2, (double)resolutionRAM);
        vdd = (resolutionCorrection * vdd - this->vdd25) / this->kVdd + 3.3;

        return vdd;
    }

    float MLX90640_GetTa(const uint16_t *frameData) const {
        float ptat;
        float ptatArt;
        float vdd;
        float ta;

        vdd = MLX90640_GetVdd(frameData);

        ptat = frameData[800];
        if (ptat > 32767) {
            ptat = ptat - 65536;
        }

        ptatArt = frameData[768];
        if (ptatArt > 32767) {
            ptatArt = ptatArt - 65536;
        }
        ptatArt =
            (ptat / (ptat * this->alphaPTAT + ptatArt)) * pow(2, (double)18);

        ta = (ptatArt / (1 + this->KvPTAT * (vdd - 3.3)) - this->vPTAT25);
        ta = ta / this->KtPTAT + 25;

        return ta;
    }

    void MLX90640_CalculateTo(
        const uint16_t *frameData, float emissivity, float tr,
        m5::MLX90640_Class::temp_data_t *result,
        const m5::MLX90640_Class::temp_data_t *prev_result,
        uint32_t filter_level) {
        result->min_info.temp = UINT16_MAX;
        result->max_info.temp = 0;
        int32_t total_temp    = 0;
        //      uint32_t diff_temp = 0;

        float irDataCP[2];
        float alphaCorrR[4];
        int8_t range;
        bool subPage = frameData[833];

        float vdd_minus_33 = MLX90640_GetVdd(frameData) - 3.3;
        float ta           = MLX90640_GetTa(frameData);

        float ta4 = (ta + 273.15f);
        ta4 *= ta4;
        ta4 *= ta4;

        float tr4 = (tr + 273.15f);
        tr4 *= tr4;
        tr4 *= tr4;

        float taTr = tr4 - (tr4 - ta4) / emissivity;

        float ktaScale   = pow(2, (double)this->ktaScale);
        float kvScale    = pow(2, (double)this->kvScale);
        float alphaScale = pow(2, (double)this->alphaScale);

        alphaCorrR[0] = 1 / (1 + this->ksTo[0] * 40);
        alphaCorrR[1] = 1;
        alphaCorrR[2] = (1 + this->ksTo[1] * this->ct[2]);
        alphaCorrR[3] =
            alphaCorrR[2] * (1 + this->ksTo[2] * (this->ct[3] - this->ct[2]));

        //------------------------- Gain calculation
        //-----------------------------------
        float gain = frameData[778];
        if (gain > 32767) {
            gain -= 65536;
        }
        gain = this->gainEE / gain;

        //------------------------- To calculation
        //-------------------------------------
        uint8_t mode = (frameData[832] & 0x1000) >> 5;

        irDataCP[0] = frameData[776];
        irDataCP[1] = frameData[808];
        for (int i = 0; i < 2; i++) {
            if (irDataCP[i] > 32767) {
                irDataCP[i] -= 65536;
            }
            irDataCP[i] *= gain;
        }
        irDataCP[0] -= this->cpOffset[0] * (1 + this->cpKta * (ta - 25)) *
                       (1 + this->cpKv * vdd_minus_33);
        if (mode == this->calibrationModeEE) {
            irDataCP[1] -= this->cpOffset[1] * (1 + this->cpKta * (ta - 25)) *
                           (1 + this->cpKv * vdd_minus_33);
        } else {
            irDataCP[1] -= (this->cpOffset[1] + this->ilChessC[0]) *
                           (1 + this->cpKta * (ta - 25)) *
                           (1 + this->cpKv * vdd_minus_33);
        }

        float ksTo127315 = 1 - this->ksTo[1] * 273.15f;

        for (int i = 0; i < 384; ++i) {
            int ilPattern   = (i >> 4) & 1;
            int pixelNumber = (i << 1) + ((ilPattern ^ subPage) & 1);
            // int chessPattern = ilPattern ^ (pixelNumber & 1);
            /*
                    if (mode == 0)
                    {
                      if (ilPattern != subPage) continue;
                    }
                    else
                    {
                      if (chessPattern != subPage) continue;
                    }
                  //*/
            // if (pixelNumber < 768)
            bool isBroken = false;

            for (int idx = 0; !isBroken && idx < 5 && brokenPixels[idx] < 768;
                 ++idx) {
                isBroken = (pixelNumber == brokenPixels[idx]);
            }
            for (int idx = 0; !isBroken && idx < 5 && outlierPixels[idx] < 768;
                 ++idx) {
                isBroken = (pixelNumber == outlierPixels[idx]);
            }

            if (!isBroken) {
                int conversionPattern =
                    (((pixelNumber + 2) >> 2) - ((pixelNumber + 3) >> 2) +
                     ((pixelNumber + 1) >> 2) - (pixelNumber >> 2)) *
                    (1 - 2 * ilPattern);

                int tmp = frameData[pixelNumber];
#if defined(DEBUG_BROKENPIXEL)
                if (pixelNumber == DEBUG_BROKENPIXEL) {
                    tmp = 0x7FFF;
                }
#endif
                if (tmp > 32767) {
                    tmp -= 65536;
                }
                float irData = gain * tmp;

                float kta = this->kta[pixelNumber] / ktaScale;
                float kv  = this->kv[pixelNumber] / kvScale;
                irData    = irData - this->offset[pixelNumber] *
                                      (1 + kta * (ta - 25)) *
                                      (1 + kv * vdd_minus_33);

                if (mode != this->calibrationModeEE) {
                    irData = irData + this->ilChessC[2] * (2 * ilPattern - 1) -
                             this->ilChessC[1] * conversionPattern;
                }

                irData = irData - this->tgc * irDataCP[subPage];
                irData = irData / emissivity;

                float alphaCompensated =
                    SCALEALPHA * alphaScale / this->alpha[pixelNumber];
                alphaCompensated =
                    alphaCompensated * (1 + this->KsTa * (ta - 25));

                float Sx = alphaCompensated * alphaCompensated *
                           alphaCompensated *
                           (irData + alphaCompensated * taTr);
                // Sx = fast_sqrt2(Sx) * this->ksTo[1];
                // float To = fast_sqrt2(irData / (alphaCompensated *
                // (ksTo127315) + Sx) + taTr) - 273.15f;
                Sx       = sqrtf(sqrtf(Sx)) * this->ksTo[1];
                float To = sqrtf(sqrtf(
                               irData / (alphaCompensated * (ksTo127315) + Sx) +
                               taTr)) -
                           273.15f;

                if (To < this->ct[1]) {
                    range = 0;
                } else if (To < this->ct[2]) {
                    range = 1;
                } else if (To < this->ct[3]) {
                    range = 2;
                } else {
                    range = 3;
                }
                auto temp = (int32_t)roundf(
                    (sqrtf(
                         sqrtf(irData / (alphaCompensated * alphaCorrR[range] *
                                         (1 + this->ksTo[range] *
                                                  (To - this->ct[range]))) +
                               taTr)) +
                     ((float)m5::MLX90640_Class::DATA_OFFSET - 273.15f)) *
                    m5::MLX90640_Class::DATA_RATIO_VALUE);
                if (temp < 0) {
                    temp = 0;
                } else if (temp > UINT16_MAX) {
                    temp = UINT16_MAX;
                }
                if (filter_level) {  /// フィルタ処理
                                     /// (前回の温度と比較して一定以上の差がないと反応させない)
                    int x = (pixelNumber & 31) - 15;
                    if (x < 0) {
                        x = ~x;
                    }
                    int y = (pixelNumber >> 5) - 13;
                    if (y < 0) {
                        y = ~y;
                    }
                    // 外周ピクセルほどノイズが多いため、ピクセル位置に応じてテーブルから補正係数を掛ける;
                    int noise_filter =
                        (filter_level * (96 + noise_tbl[x + (y * 17)])) >> 8;

                    int diff = temp - prev_result->data[i];
                    if (abs(diff) > noise_filter) {
                        temp += (diff < 0) ? noise_filter : -noise_filter;
                    } else {
                        temp = prev_result->data[i];
                        // if (diff) temp += (diff < 0) ? -1 : 1;
                    }
                }
                result->data[i] = temp;
            } else {  // 破損ピクセルの場合、前回の結果を用いて隣接ピクセル（最大４点）の平均を用いて更新する;
                int pn       = pixelNumber - 32;
                size_t x     = pn & 31;
                size_t y     = pn >> 5;
                uint32_t sum = 0;
                int count    = 0;
                if (x > 0) {
                    ++count;
                    sum += prev_result->data[(pn - 1) >> 1];
                }
                if (x < 31) {
                    ++count;
                    sum += prev_result->data[(pn + 1) >> 1];
                }
                if (y > 0) {
                    ++count;
                    sum += prev_result->data[(pn - 32) >> 1];
                }
                if (y < 23) {
                    ++count;
                    sum += prev_result->data[(pn + 32) >> 1];
                }
                result->data[i] = sum / count;
            }
        }
    }

    void MLX90640_CalculateTo(const uint16_t *frameData, float emissivity,
                              float tr, uint16_t *result) {
        float irDataCP[2];
        float alphaCorrR[4];
        int8_t range;
        bool subPage = frameData[833];

        float vdd_minus_33 = MLX90640_GetVdd(frameData) - 3.3;
        float ta           = MLX90640_GetTa(frameData);

        float ta4 = (ta + 273.15f);
        ta4 *= ta4;
        ta4 *= ta4;

        float tr4 = (tr + 273.15f);
        tr4 *= tr4;
        tr4 *= tr4;

        float taTr = tr4 - (tr4 - ta4) / emissivity;

        float ktaScale   = pow(2, (double)this->ktaScale);
        float kvScale    = pow(2, (double)this->kvScale);
        float alphaScale = pow(2, (double)this->alphaScale);

        alphaCorrR[0] = 1 / (1 + this->ksTo[0] * 40);
        alphaCorrR[1] = 1;
        alphaCorrR[2] = (1 + this->ksTo[1] * this->ct[2]);
        alphaCorrR[3] =
            alphaCorrR[2] * (1 + this->ksTo[2] * (this->ct[3] - this->ct[2]));

        //------------------------- Gain calculation
        //-----------------------------------
        float gain = frameData[778];
        if (gain > 32767) {
            gain -= 65536;
        }
        gain = this->gainEE / gain;

        //------------------------- To calculation
        //-------------------------------------
        uint8_t mode = (frameData[832] & 0x1000) >> 5;

        irDataCP[0] = frameData[776];
        irDataCP[1] = frameData[808];
        for (int i = 0; i < 2; i++) {
            if (irDataCP[i] > 32767) {
                irDataCP[i] -= 65536;
            }
            irDataCP[i] *= gain;
        }
        irDataCP[0] -= this->cpOffset[0] * (1 + this->cpKta * (ta - 25)) *
                       (1 + this->cpKv * vdd_minus_33);
        if (mode == this->calibrationModeEE) {
            irDataCP[1] -= this->cpOffset[1] * (1 + this->cpKta * (ta - 25)) *
                           (1 + this->cpKv * vdd_minus_33);
        } else {
            irDataCP[1] -= (this->cpOffset[1] + this->ilChessC[0]) *
                           (1 + this->cpKta * (ta - 25)) *
                           (1 + this->cpKv * vdd_minus_33);
        }

        float ksTo127315 = 1 - this->ksTo[1] * 273.15f;

        for (int i = 0; i < 384; ++i) {
            int ilPattern   = (i >> 4) & 1;
            int pixelNumber = (i << 1) + ((ilPattern ^ subPage) & 1);

            {
                int conversionPattern =
                    (((pixelNumber + 2) >> 2) - ((pixelNumber + 3) >> 2) +
                     ((pixelNumber + 1) >> 2) - (pixelNumber >> 2)) *
                    (1 - 2 * ilPattern);

                int tmp = frameData[pixelNumber];
#if defined(DEBUG_BROKENPIXEL)
                if (pixelNumber == DEBUG_BROKENPIXEL) {
                    tmp = 0x7FFF;
                }
#endif
                if (tmp > 32767) {
                    tmp -= 65536;
                }
                float irData = gain * tmp;

                float kta = this->kta[pixelNumber] / ktaScale;
                float kv  = this->kv[pixelNumber] / kvScale;
                irData    = irData - this->offset[pixelNumber] *
                                      (1 + kta * (ta - 25)) *
                                      (1 + kv * vdd_minus_33);

                if (mode != this->calibrationModeEE) {
                    irData = irData + this->ilChessC[2] * (2 * ilPattern - 1) -
                             this->ilChessC[1] * conversionPattern;
                }

                irData = irData - this->tgc * irDataCP[subPage];
                irData = irData / emissivity;

                float alphaCompensated =
                    SCALEALPHA * alphaScale / this->alpha[pixelNumber];
                alphaCompensated =
                    alphaCompensated * (1 + this->KsTa * (ta - 25));

                float Sx = alphaCompensated * alphaCompensated *
                           alphaCompensated *
                           (irData + alphaCompensated * taTr);
                // Sx = fast_sqrt2(Sx) * this->ksTo[1];
                // float To = fast_sqrt2(irData / (alphaCompensated *
                // (ksTo127315) + Sx) + taTr) - 273.15f;
                Sx       = sqrtf(sqrtf(Sx)) * this->ksTo[1];
                float To = sqrtf(sqrtf(
                               irData / (alphaCompensated * (ksTo127315) + Sx) +
                               taTr)) -
                           273.15f;

                if (To < this->ct[1]) {
                    range = 0;
                } else if (To < this->ct[2]) {
                    range = 1;
                } else if (To < this->ct[3]) {
                    range = 2;
                } else {
                    range = 3;
                }
                auto temp = (int32_t)roundf(
                    (sqrtf(
                         sqrtf(irData / (alphaCompensated * alphaCorrR[range] *
                                         (1 + this->ksTo[range] *
                                                  (To - this->ct[range]))) +
                               taTr)) +
                     ((float)m5::MLX90640_Class::DATA_OFFSET - 273.15f)) *
                    m5::MLX90640_Class::DATA_RATIO_VALUE);
                if (temp < 0) {
                    temp = 0;
                } else if (temp > UINT16_MAX) {
                    temp = UINT16_MAX;
                }
                result[i] = temp;
            }
        }

        // 破損ピクセル箇所の補間処理
        // (隣接ピクセル（最大４点）の平均を用いて更新する)
        for (auto bp : {brokenPixels, outlierPixels}) {
            for (int idx = 0; bp[idx] < 768 && idx < 5; ++idx) {
                auto pixelNumber = bp[idx];
                int i            = pixelNumber >> 1;
                int ilPattern    = (i >> 4) & 1;
                if (pixelNumber == ((i << 1) + ((ilPattern ^ subPage) & 1))) {
                    size_t x     = pixelNumber & 31;
                    size_t y     = pixelNumber >> 5;
                    uint32_t sum = 0;
                    int count    = 0;
                    if (x > 1) {
                        ++count;
                        sum += result[i - 1];
                    }
                    if (x < 30) {
                        ++count;
                        sum += result[i + 1];
                    }
                    if (y > 0) {
                        ++count;
                        sum += result[i - 16];
                    }
                    if (y < 23) {
                        ++count;
                        sum += result[i + 16];
                    }
                    result[i] = sum / count;
                }
            }
        }
    }
};

static MLX90640_params_t MLX90640_params;

bool MLX90640_Class::readReg(uint16_t reg, uint16_t *data, size_t len) {
    return _i2c->start(_i2c_addr, false, 400000) && _i2c->writeWords(&reg, 1) &&
           _i2c->restart(_i2c_addr, true, 400000) &&
           _i2c->readWords(data, len, true, _i2c_freq) && _i2c->stop();
}

bool MLX90640_Class::writeReg(uint16_t reg, const uint16_t *data, size_t len) {
    return _i2c->start(_i2c_addr, false, 400000) && _i2c->writeWords(&reg, 1) &&
           _i2c->writeWords(data, len) && _i2c->stop();
}

bool MLX90640_Class::writeReg(uint16_t reg, uint16_t value) {
    return writeReg(reg, &value, 1);
}

bool MLX90640_Class::init(I2C_Master *i2c) {
    _i2c = i2c;

    uint16_t data[1024];
    if (readReg(0x2400, data, 832)) {
        MLX90640_params.setParam(data);
        return true;
    }
    return false;
}

void MLX90640_Class::setRate(refresh_rate_t rate) {
    int r         = rate & 7;
    _refresh_rate = (refresh_rate_t)r;
    // If the refresh rate is 32 Hz or higher, the communication speed is
    // increased because I2C 400 kHz cannot meet the refresh cycle.
    uint32_t freq = 9375u << r;
    if (freq < 100000) {
        freq = 100000;
    }
    _i2c_freq = freq;

    uint16_t tmp;

    while (!readReg(0x800D, &tmp, 1)) {
        vTaskDelay(1);
    }

    uint16_t value = (tmp & 0xFC7F) | (_refresh_rate << 7);
    writeReg(0x800D, &value, 1);
    writeReg(0x8000, 0x0030);
}

bool MLX90640_Class::readFrameData(uint16_t *data) {
    if (!readReg(0x8000, data, 1) || !((data[0] & 0x08))) return false;
    data[833] = data[0] & 1;  // subPage

    if (readReg(0x0400, data, 832) && readReg(0x800D, &data[832], 1)) {
        // データ破損対策：830番が異常値になっていないかチェックする;
        return (data[830] < 0xFF) && writeReg(0x8000, 0x0030);
    }
    return false;
}

void MLX90640_Class::calcTempData(const uint16_t *framedata,
                                  temp_data_t *tempdata, float emissivity) {
    float Ta = MLX90640_params.MLX90640_GetTa(framedata);

    // Reflected temperature based on the sensor ambient temperature
    float tr = Ta - TA_SHIFT;

    tempdata->subpage = framedata[833];
    MLX90640_params.MLX90640_CalculateTo(framedata, emissivity, tr,
                                         tempdata->data);
}

void MLX90640_Class::calcTempData(const uint16_t *framedata,
                                  temp_data_t *tempdata,
                                  const temp_data_t *prev_tempdata,
                                  uint32_t filter_level, uint8_t monitor_width,
                                  uint8_t monitor_height) {
    float emissivity = 0.95;

    float Ta = MLX90640_params.MLX90640_GetTa(framedata);

    // Reflected temperature based on the sensor ambient temperature
    float tr = Ta - TA_SHIFT;

    bool subpage      = framedata[833];
    tempdata->subpage = subpage;
    MLX90640_params.MLX90640_CalculateTo(framedata, emissivity, tr, tempdata,
                                         prev_tempdata, filter_level);

    uint16_t data[384];
    uint16_t min_temp = UINT16_MAX;
    uint16_t max_temp = 0;
    uint16_t min_idx  = 0;
    uint16_t max_idx  = 0;

    uint32_t total_temp   = 0;
    uint_fast8_t mx       = 16 - monitor_width;
    uint_fast8_t my       = 12 - monitor_height;
    uint_fast8_t ye       = my + monitor_height * 2;
    uint_fast16_t dst_idx = 0;
    auto src              = tempdata->data;
    for (uint_fast8_t y = my; y < ye; ++y) {
        auto idx =
            y * (PIXEL_COLS >> 1) + ((mx + ((mx + y + subpage) & 1)) >> 1);
        for (uint_fast8_t i = 0; i < monitor_width; ++i, ++idx, ++dst_idx) {
            auto temp     = src[idx];
            data[dst_idx] = temp;
            total_temp += temp;
            if (min_temp > temp) {
                min_temp = temp;
                min_idx  = idx;
            }
            if (max_temp < temp) {
                max_temp = temp;
                max_idx  = idx;
            }
        }
    }
    tempdata->avg_temp = total_temp / dst_idx;

    size_t n = dst_idx / 2;
    std::nth_element(data, &data[n], &data[dst_idx]);
    tempdata->med_temp = data[n];

    tempdata->min_info.temp = min_temp;
    tempdata->max_info.temp = max_temp;
    {
        int y                = max_idx >> 4;
        tempdata->max_info.y = y;
        tempdata->max_info.x = ((max_idx & 15) << 1) + ((y ^ subpage) & 1);

        y                    = min_idx >> 4;
        tempdata->min_info.y = y;
        tempdata->min_info.x = ((min_idx & 15) << 1) + ((y ^ subpage) & 1);
    }
}
}  // namespace m5