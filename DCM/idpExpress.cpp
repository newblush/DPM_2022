/**
 * @file	idpExpress.cpp
 * @brief	Photron IDP-Express画像取得用クラスの実装
 * @date	2014/04/10
 * @author	末石 智大
 */

 #if 0
#include <windows.h>
#include <iostream>
#include "idpExpress.h"
#include <opencv2/opencv.hpp>

#pragma comment(lib, "PDCLIB.lib")

/** デフォルトコンストラクタ */
idpExpress::idpExpress(){
	/** 初期化 */
	nRet = PDC_Init(&nErrorCode);
	showError("PDC_Init");

	/** 接続されたデバイスの検索 */
	nRet = PDC_DetectDevice(PDC_INTTYPE_PCI, NULL, 1, PDC_DETECT_NORMAL, &DetectNumInfo, &nErrorCode);
	showError("PDC_DetectDevice");

	/** 検索結果が0台の場合は終了 */
	if (DetectNumInfo.m_nDeviceNum == 0){
		showError("Detect Number is 0");
		return;
	}

	/** 検索結果がIDPExpressでない場合は終了 */
	if (DetectNumInfo.m_DetectInfo[0].m_nDeviceCode != PDC_DEVTYPE_IDPEXPRESS){
		showError("Detect Device is not PDC_DEVTYPE_IDPEXPRESS");
		return;
	}

	/** デバイスのオープン */
	nRet = PDC_OpenDevice(&(DetectNumInfo.m_DetectInfo[0]), &nDeviceNo, &nErrorCode);
	showError("PDC_OpenDevice");

	/** 子デバイスの取得 */
	nRet = PDC_GetExistChildDeviceList(nDeviceNo, &nCount, ChildNo, &nErrorCode);
	showError("PDC_GetExistChildDeviceList");

	/** 現在の動作モードを取得 */
	nRet = PDC_GetStatus(nDeviceNo, &nStatus, &nErrorCode);
	showError("PDC_GetStatus");

	/** メモリ再生モードの場合はライブモードに切り替える */
	if (nStatus == PDC_STATUS_PLAYBACK){
		nRet = PDC_SetStatus(nDeviceNo, PDC_STATUS_LIVE, &nErrorCode);
		showError("PDC_SetStatus");
	}

	for (int i = 0; i < (int)nCount; i++){
		/** 撮影速度の設定 */
		nRet = PDC_SetRecordRate(nDeviceNo, ChildNo[i], FRAME_PER_SEC, &nErrorCode);
		showError("PDC_SetRecordRate");

		/** 撮影解像度の設定 */
		nRet = PDC_SetResolution(nDeviceNo, ChildNo[i], CAMERA_WIDTH, CAMERA_HEIGHT, &nErrorCode);
		showError("PDC_SetResolution");

		/** センサーゲイン増幅の設定 */
		nRet = PDC_SetSensorGainMode(nDeviceNo, ChildNo[i], PDC_SENSOR_GAIN_X1, &nErrorCode);
		showError("PDC_SetSensorGainMode");

		/** ベイヤー転送を有効化 */
		nRet = PDC_SetTransferOption(nDeviceNo, ChildNo[i], PDC_8BITSEL_8NORMAL, PDC_FUNCTION_ON, PDC_COLORDATA_INTERLEAVE_BGR, &nErrorCode);
		showError("PDC_SetTransferOption");

		/** ライブ画像取得可能か確認 */
		unsigned char *pBuf = (unsigned char*)malloc(CAMERA_WIDTH * CAMERA_HEIGHT);
		nRet = PDC_GetLiveImageData(nDeviceNo, ChildNo[i], 8, pBuf, &nErrorCode);
		showError("PDC_GetLiveImageData");
		free(pBuf);
	}

	/**
	 * センタートリガに設定
	 * @attention PDC_GetLiveImageAddress2を呼んだときににnFrameNoが更新されるようになる
	 */
	nRet = PDC_SetTriggerMode(nDeviceNo, PDC_TRIGGER_CENTER, 0, 0, 0, &nErrorCode);
	showError("PDC_SetTriggerMode");

	/** 録画準備状態に設定 */
	nRet = PDC_SetRecReady(nDeviceNo, &nErrorCode);
	showError("PDC_SetRecReady");

	/** エンドレス録画状態に */
	nRet = PDC_SetEndless(nDeviceNo, &nErrorCode);
	showError("PDC_SetEndless");

	/** カメラ起動まで待つ */
	Sleep(500);
}

/** デストラクタ */
idpExpress::~idpExpress(){
	/** デバイスクローズ */
	nRet = PDC_CloseDevice(nDeviceNo, &nErrorCode);
	showError("PDC_CloseDevice");
}

//-----------------------------------------------------------------------------
// Private Method
//-----------------------------------------------------------------------------
/**
 * @brief エラー確認
 * 
 * @param[in] str 関数名
 * @return エラーがなければ0, あれば1
 */
int idpExpress::showError(std::string str){
	std::stringstream message;
	message << str << " Error " << nErrorCode;
	if (nRet == PDC_FAILED){
		std::cout << message.str() << std::endl;
		return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------------
// Public Method
//-----------------------------------------------------------------------------
/**
 * @brief SSEを利用した画像分割(単眼取得用)
 *
 * @param[in] data			画像情報を格納しているメモリアドレス
 * @param[in, out] image	取り込む画像(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 */
void idpExpress::devideSSE(unsigned char *data, cv::Mat image){
	__m128i a;
	int j=0;
	int size = 2 * CAMERA_HEIGHT * CAMERA_WIDTH;

	for(int i=0; i<size;i+=32){
		__m128i in1 = _mm_loadu_si128((__m128i*)(data+i));
		__m128i in2 = _mm_loadu_si128((__m128i*)(data+i+16));

		a = _mm_unpacklo_epi64(in1,in2);

		j=i/2;
		_mm_storeu_si128((__m128i*)(image.data+j),a);
	}
}

/**
 * @brief SSEを利用した画像分割(ステレオ取得用)
 *
 * @param[in] data			画像情報を格納しているメモリアドレス
 * @param[in, out] image1	取り込む画像(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @param[in, out] image2	取り込む画像(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 */
void idpExpress::devideSSEStereo(unsigned char *data, cv::Mat image1, cv::Mat image2){
	__m128i a, b;
	int j=0;
	int size = 2 * CAMERA_HEIGHT * CAMERA_WIDTH;

	for(int i=0; i<size;i+=32){
		__m128i in1 = _mm_loadu_si128((__m128i*)(data+i));
		__m128i in2 = _mm_loadu_si128((__m128i*)(data+i+16));

		b = _mm_unpackhi_epi64(in1,in2);
		a = _mm_unpacklo_epi64(in1,in2);

		j=i/2;
		_mm_storeu_si128((__m128i*)(image1.data+j),a);
		_mm_storeu_si128((__m128i*)(image2.data+j),b);
	}
}


/**
 * @brief 単眼画像取得
 * 
 * @param[in, out] image 取り込む画像(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @return 画像取得成功時はtrue，失敗時はfalse
 */
bool idpExpress::getFrame(cv::Mat image){
	void  *baseAdress;
	unsigned char *pSrc, *pDst;
	if (PDC_GetLiveImageAddress2(nDeviceNo, ChildNo[0], &nFrameNo, &baseAdress, &nErrorCode) != PDC_SUCCEEDED)
		return false;
	if (nOldFrameNo != 0 && nFrameNo == nOldFrameNo) return false;
	nOldFrameNo = nFrameNo;

#if USE_BAYER_TRANSFORM
	cv::Mat imageBayer(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1);
#endif
	if (nCount == 1){
		pSrc = (unsigned char *)baseAdress + 8; // 8 for header, 8 for second camera
#if !USE_BAYER_TRANSFORM
		pDst = image.data;
#else
		pDst = imageBayer.data;
#endif
		memcpy(pDst, pSrc, CAMERA_WIDTH * CAMERA_HEIGHT * sizeof(unsigned char));
	}
	else if (nCount == 2){
		if(USC_CAM_HEAD_ID == 0){
			pSrc = (unsigned char *)baseAdress + 8 + 8; // 8 for header, 8 for second camera
		} else { 
			pSrc = (unsigned char *)baseAdress + 8; // 8 for header, 8 for second camera
		}
#if USE_SSE
		devideSSE(pSrc, image);
#else
		for (int y = 0; y < CAMERA_HEIGHT; y++){
#if !USE_BAYER_TRANSFORM
			pDst = (unsigned char *)&(image.data[image.step*y]);
#else
			pDst = (unsigned char *)&(imageBayer.data[imageBayer.step*y]);
#endif
			for (int x = 0; x < CAMERA_WIDTH; x = x + 8){
				memcpy(pDst, pSrc, 8 * sizeof(unsigned char));
				pDst += 8; pSrc += 16;
			}
		}
#endif
	}
	else {
		return false;
	}

#if USE_BAYER_TRANSFORM
	cv::cvtColor(imageBayer, image, CV_BayerGB2GRAY);
#endif

	return true;
}

/**
 * @brief ROIを指定した単眼画像取得
 * 
 * @param[in, out] image	取り込む画像(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @param[in] x_start		X方向開始位置
 * @param[in] x_end		X方向終了位置
 * @param[in] y_start	Y方向開始位置
 * @param[in] y_end		Y方向終了位置
 * @return 画像取得成功時はtrue，失敗時はfalse
 */
bool idpExpress::getFrameROI(cv::Mat image, int x_start, int x_end, int y_start, int y_end){
	void  *baseAdress;
	unsigned char *pSrc, *pDst;
	if (PDC_GetLiveImageAddress2(nDeviceNo, ChildNo[0], &nFrameNo, &baseAdress, &nErrorCode) != PDC_SUCCEEDED)
		return false;
	if (nOldFrameNo != 0 && nFrameNo == nOldFrameNo) return false;
	nOldFrameNo = nFrameNo;

	// 接続カメラヘッド台数確認
	if (nCount == 1){
		pSrc = (unsigned char *)baseAdress + 8; // 8 for header, 8 for second camera
		for (int y = 0; y < CAMERA_HEIGHT; y++){
			pDst = (unsigned char *)&(image.data[image.step*y]);
			for (int x = 0; x < CAMERA_WIDTH; x = x + 8){
				if (x > x_start && x < x_end && y > y_start && y < y_end)
					memcpy(pDst, pSrc, 8 * sizeof(unsigned char));
				pDst += 8; pSrc += 8;
			}
		}
	}
	else if (nCount == 2){
		if(USC_CAM_HEAD_ID == 0)
			pSrc = (unsigned char *)baseAdress + 8 + 8; // 8 for header, 8 for second camera
		else 
			pSrc = (unsigned char *)baseAdress + 8; // 8 for header, 8 for second camera
		for (int y = 0; y < CAMERA_HEIGHT; y++){
			pDst = (unsigned char *)&(image.data[image.step*y]);
			for (int x = 0; x < CAMERA_WIDTH; x = x + 8){
				if (x > x_start && x < x_end && y > y_start && y < y_end)
					memcpy(pDst, pSrc, 8 * sizeof(unsigned char));
				pDst += 8; pSrc += 16;
			}
		}
	}
	else {
		return false;
	}

	return true;
}

/**
 * @brief ステレオ画像取得
 * 
 * @param[in, out] image1 取り込む画像1(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @param[in, out] image2 取り込む画像2(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @return 画像取得成功時はtrue，失敗時はfalse
 */
bool idpExpress::getFrameStereo(cv::Mat image1, cv::Mat image2){
	// 接続カメラヘッド数が2台か確認
	if (nCount != 2) return false;

	void  *baseAdress;
	unsigned char *pSrc, *pDst1, *pDst2;
	if (PDC_GetLiveImageAddress2(nDeviceNo, ChildNo[0], &nFrameNo, &baseAdress, &nErrorCode) != PDC_SUCCEEDED)
		return false;
	if (nOldFrameNo != 0 && nFrameNo == nOldFrameNo) return false;
	nOldFrameNo = nFrameNo;

	pSrc = (unsigned char *)baseAdress + 8; // 8 for header
#if USE_SSE
	devideSSEStereo(pSrc, image1, image2);
#else
	for (int y = 0; y < CAMERA_HEIGHT; y++){
		pDst1 = (unsigned char *)&(image1.data[image1.step*y]);
		pDst2 = (unsigned char *)&(image2.data[image2.step*y]);
		for (int x = 0; x < CAMERA_WIDTH; x = x + 8){
			memcpy(pDst2, pSrc, 8 * sizeof(unsigned char));
			pDst2 += 8; pSrc += 8;
			memcpy(pDst1, pSrc, 8 * sizeof(unsigned char));
			pDst1 += 8; pSrc += 8;
		}
	}
#endif
	return true;
}

/**
 * @brief ROIを指定したステレオ画像取得
 * 
 * @param[in, out] image1 取り込む画像1(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @param[in, out] image2 取り込む画像2(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)である必要アリ)
 * @param[in] x_start		X方向開始位置
 * @param[in] x_end		X方向終了位置
 * @param[in] y_start	Y方向開始位置
 * @param[in] y_end		Y方向終了位置
 * @return 画像取得成功時はtrue，失敗時はfalse
 */
bool idpExpress::getFrameStereoROI(cv::Mat image1, cv::Mat image2, int x_start, int x_end, int y_start, int y_end){
	// 接続カメラヘッド数が2台か確認
	if (nCount != 2) return false;

	void  *baseAdress;
	unsigned char *pSrc, *pDst1, *pDst2;
	if (PDC_GetLiveImageAddress2(nDeviceNo, ChildNo[0], &nFrameNo, &baseAdress, &nErrorCode) != PDC_SUCCEEDED)
		return false;
	if (nOldFrameNo != 0 && nFrameNo == nOldFrameNo) return false;
	nOldFrameNo = nFrameNo;

	pSrc = (unsigned char *)baseAdress + 8; // 8 for header
	for (int y = 0; y < CAMERA_HEIGHT; y++){
		pDst1 = (unsigned char *)&(image1.data[image1.step*y]);
		pDst2 = (unsigned char *)&(image2.data[image2.step*y]);
		for (int x = 0; x < CAMERA_WIDTH; x = x + 8){
			if (x > x_start && x < x_end && y > y_start && y < y_end)
				memcpy(pDst2, pSrc, 8 * sizeof(unsigned char));
			pDst2 += 8; pSrc += 8;
			if (x > x_start && x < x_end && y > y_start && y < y_end)
				memcpy(pDst1, pSrc, 8 * sizeof(unsigned char));
			pDst1 += 8; pSrc += 8;
		}
	}

	return true;
}
#endif