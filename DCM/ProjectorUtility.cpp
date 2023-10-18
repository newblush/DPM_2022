#include "ProjectorUtility.h"


DYNAFLASH_PARAM getDefaultDynaParam() {
    DYNAFLASH_PARAM param;
    param.dBProportion = 33.4;  // 0.00 - 100 Proportion
    param.dGProportion = 33.3;  // 0.00 - 100 Proportion
    param.dRProportion = 33.3;  // 0.00 - 100 Proportion
    param.dWProportion = 0.0;   // 0.00 - 100 Proportion
    param.dFrameRate = 946.0;   // Binary Mode: 0 - 22000; GrayMode: 1 - 2840 RGB Mode : fps 0.03 fps - 946 fps;

    param.nBinaryMode = FRAME_MODE_RGB;  // binary = 0, Gray = 1, RGB = 2, RGBW = 3
    param.nBitDepth = 8;                 // 8: [7:0], 7: [7:1], 6: [7:2], 5: [7:3], 4: [7:4], 3: [7:5], 2: [7:6], 1: [7:7]
    param.nMirrorMode = 0;               // 0: enable 1: disable
    param.nFlipMode = 0;                 // 0: enable 1: disable
    param.nCompData = 0;                 // 0: enable 1: disable
    param.nBlockNum = 0;                 // 0: Block 16 1: Block 14
    param.nTriggerSelect = 0;            // 0: Internal 1: External
    param.nTriggerSkip = 0;              // 0: disable 1: enable
    param.nTimingSel = 0;                // 0: auto calculate 1: manual
    param.nTimingMode = 0;               // 0: Normal Mode    1: Timing Mode

    for (int i = 0; i < 4; i++) {
        // RGBW LED ON/OFF Timing
        param.nSWCount[i][0] = 5000;
        param.nSWCount[i][1] = 4000;
        param.nSWCount[i][2] = 7000;
        param.nSWCount[i][3] = 2000;
        param.nSWCount[i][4] = 8000;
        param.nSWCount[i][5] = 1000;
        param.nSWCount[i][6] = 8500;
        param.nSWCount[i][7] = 500;
        param.nSWCount[i][8] = 8750;
        param.nSWCount[i][9] = 250;
        param.nSWCount[i][10] = 8875;
        param.nSWCount[i][11] = 125;
        param.nSWCount[i][12] = 8937;
        param.nSWCount[i][13] = 63;
        param.nSWCount[i][14] = 1000;
        param.nSWCount[i][15] = 8000;
        param.nSWCount[i][16] = 0;
        param.nSWCount[i][17] = 0;
        param.nSWCount[i][18] = 0;
        param.nSWCount[i][19] = 0;

        // RGBW Global Reset Timing,read
        param.nGRSTOffset[i][0] = 9000;
        param.nGRSTOffset[i][1] = 9000;
        param.nGRSTOffset[i][2] = 9000;
        param.nGRSTOffset[i][3] = 9000;
        param.nGRSTOffset[i][4] = 9000;
        param.nGRSTOffset[i][5] = 9000;
        param.nGRSTOffset[i][6] = 9000;
        param.nGRSTOffset[i][7] = 9000;
    }
    return param;
}

DYNAFLASH_PARAM getDefaultDynaParamBinary() {
    DYNAFLASH_PARAM param = getDefaultDynaParam();
    param.nBinaryMode = FRAME_MODE_BINARY;  // BINARY = 0
    param.dBProportion = 0.0;
    param.dGProportion = 0.0;
    param.dRProportion = 0.0;
    param.dWProportion = 100.0;
    return param;
}

DYNAFLASH_PARAM getDefaultDynaParamGray() {
    DYNAFLASH_PARAM param = getDefaultDynaParam();
    param.nBinaryMode = FRAME_MODE_GRAY;  // GRAY = 1
    param.dBProportion = 0.0;
    param.dGProportion = 0.0;
    param.dRProportion = 0.0;
    param.dWProportion = 100.0;
    return param;
}

DYNAFLASH_PARAM getDefaultDynaParamRGB() {
    DYNAFLASH_PARAM param = getDefaultDynaParam();
    param.nBinaryMode = FRAME_MODE_RGB;  // RGB = 2
    param.dBProportion = 33.4;
    param.dGProportion = 33.3;
    param.dRProportion = 33.3;
    param.dWProportion = 0;
    return param;
}

DYNAFLASH_PARAM getDefaultDynaParamRGBW() {
    DYNAFLASH_PARAM param = getDefaultDynaParam();
    param.nBinaryMode = FRAME_MODE_RGBW;  // RGBW = 3
    param.dBProportion = 25.0;
    param.dGProportion = 25.0;
    param.dRProportion = 25.0;
    param.dWProportion = 25.0;
    return param;
}

void printDynaParam(DYNAFLASH_PARAM param) {
    std::cout << "\033[36m" << std::endl;
    switch (param.nBinaryMode) {
        case FRAME_MODE_BINARY:
            std::cout << "FRAME MODE : BINARY" << std::endl;
            break;
        case FRAME_MODE_GRAY:
            std::cout << "FRAME MODE : GRAY" << std::endl;
            break;
        case FRAME_MODE_RGB:
            std::cout << "FRAME MODE : RGB" << std::endl;
            break;
        case FRAME_MODE_RGBW:
            std::cout << "FRAME MODE : RGBW" << std::endl;
            break;
    }

    switch (param.nBinaryMode) {
        case FRAME_MODE_BINARY:
        case FRAME_MODE_GRAY:
            break;
        case FRAME_MODE_RGB:
            std::cout << "Proportion [ B : G : R ] =";
            std::cout << " [ " << param.dBProportion;
            std::cout << " : " << param.dGProportion;
            std::cout << " : " << param.dRProportion << " ]" << std::endl;
            break;
        case FRAME_MODE_RGBW:
            std::cout << "Proportion [ B : G : R : W] =";
            std::cout << " [ " << param.dBProportion;
            std::cout << " : " << param.dGProportion;
            std::cout << " : " << param.dRProportion;
            std::cout << " : " << param.dWProportion << " ]" << std::endl;
            break;
    }

    if (param.nMirrorMode == 0)
        std::cout << "MIRROR MODE  : ON" << std::endl;
    else
        std::cout << "MIRROR MODE  : OFF" << std::endl;

    if (param.nFlipMode == 0)
        std::cout << "FLIP MODE    : ON" << std::endl;
    else
        std::cout << "FLIP MODE    : OFF" << std::endl;

    if (param.nCompData == 0)
        std::cout << "COMP MODE    : ON" << std::endl;
    else
        std::cout << "COMP MODE    : OFF" << std::endl;

    if (param.nTriggerSelect == 0)
        std::cout << "TRIGGER MODE : Internal" << std::endl;
    else
        std::cout << "TRIGGER MODE : External" << std::endl;

    if (param.nTriggerSkip != 0)
        std::cout << "TRIGGER_SKIP : ON" << std::endl;
    else
        std::cout << "TRIGGER_SKIP : OFF" << std::endl;

    std::cout << "FRAME_RATE = " << param.dFrameRate << " [fps] " << std::endl;

    if (param.nTimingSel == 1) {
        if (param.nBinaryMode == FRAME_MODE_RGB || param.nBinaryMode == FRAME_MODE_RGBW) {
            std::cout << "R: {" << std::endl;
            std::cout << "  BIT_SEQUENCE = [ ";
            for (auto tmp : param.nSWCount[0]) std::cout << tmp << ", ";
            std::cout << " ]" << std::endl;
            std::cout << "  PROJECTOR_LED_ADJUST = [ ";
            for (int j = 0; j < 8; j++) std::cout << param.nGRSTOffset[0][j] << ", ";
            std::cout << " ]" << std::endl;
            std::cout << " }" << std::endl;

            std::cout << "G: {" << std::endl;
            std::cout << "  BIT_SEQUENCE = [ ";
            for (auto tmp : param.nSWCount[0]) std::cout << tmp << ", ";
            std::cout << " ]" << std::endl;
            std::cout << "  PROJECTOR_LED_ADJUST = [ ";
            for (int j = 0; j < 8; j++) std::cout << param.nGRSTOffset[0][j] << ", ";
            std::cout << " ]" << std::endl;
            std::cout << " }" << std::endl;

            std::cout << "B: {" << std::endl;
            std::cout << "  BIT_SEQUENCE = [ ";
            for (auto tmp : param.nSWCount[0]) std::cout << tmp << ", ";
            std::cout << " ]" << std::endl;
            std::cout << "  PROJECTOR_LED_ADJUST = [ ";
            for (int j = 0; j < 8; j++) std::cout << param.nGRSTOffset[0][j] << ", ";
            std::cout << " ]" << std::endl;
            std::cout << " }" << std::endl;
        }

        if (param.nBinaryMode == FRAME_MODE_BINARY || param.nBinaryMode == FRAME_MODE_GRAY || param.nBinaryMode == FRAME_MODE_RGBW) {
            std::cout << "W: {" << std::endl;
            std::cout << "  BIT_SEQUENCE = [ ";
            for (auto tmp : param.nSWCount[0]) std::cout << tmp << ", ";
            std::cout << " ]" << std::endl;
            std::cout << "  PROJECTOR_LED_ADJUST = [ ";
            for (int j = 0; j < 8; j++) std::cout << param.nGRSTOffset[0][j] << ", ";
            std::cout << " ]" << std::endl;
            std::cout << " }" << std::endl;
        }
    }
    std::cout << "\033[39m" << std::endl;
}

void convertToBinaryRow(const void *in_data, unsigned char *out_data) {
    auto ptr = ((unsigned char *)in_data);

    //以下でバイナリー列に変換
    unsigned char tmp = (unsigned char)0;  //8bit列

    for (int i = 0; i < FRAME_BUF_SIZE_BINARY; i++) {
        for (int j = 0; j < 8; j++) {
            tmp = tmp << 1;
            tmp += ((unsigned char *)in_data)[8 * i + j] != 0;
        }
        out_data[i] = tmp;
        tmp = (unsigned char)0;
    }
}
