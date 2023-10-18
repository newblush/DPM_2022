/**
 * @file	idpExpress.cpp
 * @brief	Photron IDP-Express�摜�擾�p�N���X�̎���
 * @date	2014/04/10
 * @author	���� �q��
 */

 #if 0
#include <windows.h>
#include <iostream>
#include "idpExpress.h"
#include <opencv2/opencv.hpp>

#pragma comment(lib, "PDCLIB.lib")

/** �f�t�H���g�R���X�g���N�^ */
idpExpress::idpExpress(){
	/** ������ */
	nRet = PDC_Init(&nErrorCode);
	showError("PDC_Init");

	/** �ڑ����ꂽ�f�o�C�X�̌��� */
	nRet = PDC_DetectDevice(PDC_INTTYPE_PCI, NULL, 1, PDC_DETECT_NORMAL, &DetectNumInfo, &nErrorCode);
	showError("PDC_DetectDevice");

	/** �������ʂ�0��̏ꍇ�͏I�� */
	if (DetectNumInfo.m_nDeviceNum == 0){
		showError("Detect Number is 0");
		return;
	}

	/** �������ʂ�IDPExpress�łȂ��ꍇ�͏I�� */
	if (DetectNumInfo.m_DetectInfo[0].m_nDeviceCode != PDC_DEVTYPE_IDPEXPRESS){
		showError("Detect Device is not PDC_DEVTYPE_IDPEXPRESS");
		return;
	}

	/** �f�o�C�X�̃I�[�v�� */
	nRet = PDC_OpenDevice(&(DetectNumInfo.m_DetectInfo[0]), &nDeviceNo, &nErrorCode);
	showError("PDC_OpenDevice");

	/** �q�f�o�C�X�̎擾 */
	nRet = PDC_GetExistChildDeviceList(nDeviceNo, &nCount, ChildNo, &nErrorCode);
	showError("PDC_GetExistChildDeviceList");

	/** ���݂̓��샂�[�h���擾 */
	nRet = PDC_GetStatus(nDeviceNo, &nStatus, &nErrorCode);
	showError("PDC_GetStatus");

	/** �������Đ����[�h�̏ꍇ�̓��C�u���[�h�ɐ؂�ւ��� */
	if (nStatus == PDC_STATUS_PLAYBACK){
		nRet = PDC_SetStatus(nDeviceNo, PDC_STATUS_LIVE, &nErrorCode);
		showError("PDC_SetStatus");
	}

	for (int i = 0; i < (int)nCount; i++){
		/** �B�e���x�̐ݒ� */
		nRet = PDC_SetRecordRate(nDeviceNo, ChildNo[i], FRAME_PER_SEC, &nErrorCode);
		showError("PDC_SetRecordRate");

		/** �B�e�𑜓x�̐ݒ� */
		nRet = PDC_SetResolution(nDeviceNo, ChildNo[i], CAMERA_WIDTH, CAMERA_HEIGHT, &nErrorCode);
		showError("PDC_SetResolution");

		/** �Z���T�[�Q�C�������̐ݒ� */
		nRet = PDC_SetSensorGainMode(nDeviceNo, ChildNo[i], PDC_SENSOR_GAIN_X1, &nErrorCode);
		showError("PDC_SetSensorGainMode");

		/** �x�C���[�]����L���� */
		nRet = PDC_SetTransferOption(nDeviceNo, ChildNo[i], PDC_8BITSEL_8NORMAL, PDC_FUNCTION_ON, PDC_COLORDATA_INTERLEAVE_BGR, &nErrorCode);
		showError("PDC_SetTransferOption");

		/** ���C�u�摜�擾�\���m�F */
		unsigned char *pBuf = (unsigned char*)malloc(CAMERA_WIDTH * CAMERA_HEIGHT);
		nRet = PDC_GetLiveImageData(nDeviceNo, ChildNo[i], 8, pBuf, &nErrorCode);
		showError("PDC_GetLiveImageData");
		free(pBuf);
	}

	/**
	 * �Z���^�[�g���K�ɐݒ�
	 * @attention PDC_GetLiveImageAddress2���Ă񂾂Ƃ��ɂ�nFrameNo���X�V�����悤�ɂȂ�
	 */
	nRet = PDC_SetTriggerMode(nDeviceNo, PDC_TRIGGER_CENTER, 0, 0, 0, &nErrorCode);
	showError("PDC_SetTriggerMode");

	/** �^�揀����Ԃɐݒ� */
	nRet = PDC_SetRecReady(nDeviceNo, &nErrorCode);
	showError("PDC_SetRecReady");

	/** �G���h���X�^���Ԃ� */
	nRet = PDC_SetEndless(nDeviceNo, &nErrorCode);
	showError("PDC_SetEndless");

	/** �J�����N���܂ő҂� */
	Sleep(500);
}

/** �f�X�g���N�^ */
idpExpress::~idpExpress(){
	/** �f�o�C�X�N���[�Y */
	nRet = PDC_CloseDevice(nDeviceNo, &nErrorCode);
	showError("PDC_CloseDevice");
}

//-----------------------------------------------------------------------------
// Private Method
//-----------------------------------------------------------------------------
/**
 * @brief �G���[�m�F
 * 
 * @param[in] str �֐���
 * @return �G���[���Ȃ����0, �����1
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
 * @brief SSE�𗘗p�����摜����(�P��擾�p)
 *
 * @param[in] data			�摜�����i�[���Ă��郁�����A�h���X
 * @param[in, out] image	��荞�މ摜(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
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
 * @brief SSE�𗘗p�����摜����(�X�e���I�擾�p)
 *
 * @param[in] data			�摜�����i�[���Ă��郁�����A�h���X
 * @param[in, out] image1	��荞�މ摜(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @param[in, out] image2	��荞�މ摜(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
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
 * @brief �P��摜�擾
 * 
 * @param[in, out] image ��荞�މ摜(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @return �摜�擾��������true�C���s����false
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
 * @brief ROI���w�肵���P��摜�擾
 * 
 * @param[in, out] image	��荞�މ摜(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @param[in] x_start		X�����J�n�ʒu
 * @param[in] x_end		X�����I���ʒu
 * @param[in] y_start	Y�����J�n�ʒu
 * @param[in] y_end		Y�����I���ʒu
 * @return �摜�擾��������true�C���s����false
 */
bool idpExpress::getFrameROI(cv::Mat image, int x_start, int x_end, int y_start, int y_end){
	void  *baseAdress;
	unsigned char *pSrc, *pDst;
	if (PDC_GetLiveImageAddress2(nDeviceNo, ChildNo[0], &nFrameNo, &baseAdress, &nErrorCode) != PDC_SUCCEEDED)
		return false;
	if (nOldFrameNo != 0 && nFrameNo == nOldFrameNo) return false;
	nOldFrameNo = nFrameNo;

	// �ڑ��J�����w�b�h�䐔�m�F
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
 * @brief �X�e���I�摜�擾
 * 
 * @param[in, out] image1 ��荞�މ摜1(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @param[in, out] image2 ��荞�މ摜2(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @return �摜�擾��������true�C���s����false
 */
bool idpExpress::getFrameStereo(cv::Mat image1, cv::Mat image2){
	// �ڑ��J�����w�b�h����2�䂩�m�F
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
 * @brief ROI���w�肵���X�e���I�摜�擾
 * 
 * @param[in, out] image1 ��荞�މ摜1(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @param[in, out] image2 ��荞�މ摜2(cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1)�ł���K�v�A��)
 * @param[in] x_start		X�����J�n�ʒu
 * @param[in] x_end		X�����I���ʒu
 * @param[in] y_start	Y�����J�n�ʒu
 * @param[in] y_end		Y�����I���ʒu
 * @return �摜�擾��������true�C���s����false
 */
bool idpExpress::getFrameStereoROI(cv::Mat image1, cv::Mat image2, int x_start, int x_end, int y_start, int y_end){
	// �ڑ��J�����w�b�h����2�䂩�m�F
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