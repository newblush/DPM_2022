#pragma once
#include <iostream>
#include "DynaFlash.h"

DYNAFLASH_PARAM getDefaultDynaParam();
DYNAFLASH_PARAM getDefaultDynaParamBinary();
DYNAFLASH_PARAM getDefaultDynaParamGray();
DYNAFLASH_PARAM getDefaultDynaParamRGB();
DYNAFLASH_PARAM getDefaultDynaParamRGBW();
void printDynaParam(DYNAFLASH_PARAM param);

void convertToBinaryRow(const void *in_data, unsigned char *out_data);