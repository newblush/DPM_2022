#pragma once
#include <Windows.h>

#include <iostream>
#include <stdexcept>

#include "DynaFlash.h"

#ifdef _DEBUG
#pragma comment(lib, "DynaFlash200")
#else
#pragma comment(lib, "DynaFlash200")
#endif

#define DYNA_TYPE_1 (0x1000) /* 旧DynaFlash */
#define DYNA_TYPE_2 (0x3000) /* 製品版DynaFlash */
#define DYNA_TYPE_3 (0x2000) /* カラー版DynaFlash */
#define DYNA_TYPE_4 (0x4000) /* HGrayカラー版DynaFlash */

/*
    以下の順番で実行する
	   connect
  ( -> setFrameBufferNum )
	-> setParam
	-> start
	-> sendImage
	-> stop
	-> disconnect
*/

class HighSpeedProjector {
   public:
    const int height = 768;
    const int width = 1024;

   private:
    int deviceId = 0;         // PCに接続されたDynaFlashのインデックス
    int frameBufferNum = 16;  // 確保するフレームバッファ数 (ドライバ)
    int frameSize = FRAME_BUF_SIZE_24BIT;
    int frameMode = FRAME_MODE_RGB;
    int dynaFrashType = NULL;
    unsigned long nFrameCnt = 0;

    CDynaFlash *pDynaFlash = nullptr;
    char *pBuf = nullptr;
    DYNAFLASH_STATUS stDynaFlashStatus;

   public:
    HighSpeedProjector();
    ~HighSpeedProjector();
    void destruct();

    void connect(int deviceId = 0);
    void start();
    void stop();
    void disconnect();

    // projection
    int setImage(const void *data, int frameCnt);
    int sendImage(const void *data);
    int sendImage(void);

    // parameter
    int setFrameBufferNum(int bufferNum);
    int getFrameBufferNum();
    int getFrameSize();
    int getDeviceId();
    void setParam(DYNAFLASH_PARAM param);
    int checkParam(DYNAFLASH_PARAM param);

    // status
    void printVersion();
    DYNAFLASH_STATUS getStatus();
};