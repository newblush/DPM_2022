/**
 * @file	idpExpress.h
 * @brief	Photron IDP-Express画像取得用クラス
 * @date	2014/04/25
 * @author	末石 智大
 * @par		デバイスと子デバイス
 *			デバイスはPCI-Expressのボード，子デバイスは各カメラヘッドと考えると分かりやすい
 * @par		History
 * - 20XX/XX/XX 石井抱?or奥寛雅?or奥村光平?
 *   - 初期バージョン
 *   - 旧idpe512.h
 * - 2013/11/21 末石 智大
 *   - 64bitPC移行に伴ってidpExpress.h作成
 * - 2014/04/10 末石 智大
 *   - Doxygenコメント追加
 *   - コンストラクタ・関数の整理
 * - 2014/04/25 末石 智大
 *   - SSE処理追加
 * - 2014/11/20 成田岳
 *   - USE_ONLINE等不要部分を削除
 * - 2015/11/18 成田岳
 *   - USE_BAYER_TRANSFORMを追加し，getFrameにベイヤー変換を追加
 * - 2015/12/11 成田岳 
 *   - getFrame, getFrameROI, getFrameStereo, getFrameStereoROIの戻り値をboolに変更
 */

//-----------------------------------------------------------------------------
// Header and Library
//-----------------------------------------------------------------------------

#pragma once
#include "PDCLIB.h"
#include <opencv2/opencv.hpp>

//-----------------------------------------------------------------------------
// Macro
//-----------------------------------------------------------------------------
/** フレームレート[fps](50, 60, 125, 250, 500, 1000から選択可能) */
#define FRAME_PER_SEC			1000
/** カメラヘッドID(2台接続時に1台だけ使用する場合, 0or1) */
#define USC_CAM_HEAD_ID			0
/** 画像サイズ横 */
#define CAMERA_WIDTH				512
/** 画像サイズ縦 */
#define CAMERA_HEIGHT				512
/** ステレオ接続時にSSEを用いた高速化をするかどうか(1は使う, 0は使わない) */
#define USE_SSE					0
/** getFrame()関数でベイヤー変換を行うかどうか (1は使う, 0は使わない) */
#define USE_BAYER_TRANSFORM 0


//-----------------------------------------------------------------------------
// Class
//-----------------------------------------------------------------------------
/**
 * @class	idpExpress
 * @brief	Photron IDP-Express画像取得用クラス
 */
class idpExpress{

private:
	/** 接続されたデバイスの検索結果 */
	PDC_DETECT_NUM_INFO DetectNumInfo;
	/** デバイス番号 */
	unsigned long nDeviceNo;
	/** デバイスの状態 */
	unsigned long nStatus;
	/** 子デバイスの数 */
	unsigned long nCount;
	/** 子デバイス番号 */
	unsigned long ChildNo[PDC_MAX_DEVICE];
	/** 取得した画像の番号 */
	unsigned long nFrameNo;
	/** 一つ前の画像の番号 */
	unsigned long nOldFrameNo;
	/** 関数成否判定変数 */
	unsigned long nRet;
	/** エラー番号 */
	unsigned long nErrorCode;

	int showError(std::string str);

public:
	idpExpress();
	~idpExpress();

	void devideSSE(unsigned char *data, cv::Mat image);
	void devideSSEStereo(unsigned char *data, cv::Mat image1, cv::Mat image2);
	bool getFrame(cv::Mat image);
	bool getFrameROI(cv::Mat image, int x_start, int x_end, int y_start, int y_end);
	bool getFrameStereo(cv::Mat image1, cv::Mat image2);
	bool getFrameStereoROI(cv::Mat image1, cv::Mat image2, int x_start, int x_end, int y_start, int y_end);
};