/**
 * @file	idpExpress.h
 * @brief	Photron IDP-Express�摜�擾�p�N���X
 * @date	2014/04/25
 * @author	���� �q��
 * @par		�f�o�C�X�Ǝq�f�o�C�X
 *			�f�o�C�X��PCI-Express�̃{�[�h�C�q�f�o�C�X�͊e�J�����w�b�h�ƍl����ƕ�����₷��
 * @par		History
 * - 20XX/XX/XX �Έ��?or������?or��������?
 *   - �����o�[�W����
 *   - ��idpe512.h
 * - 2013/11/21 ���� �q��
 *   - 64bitPC�ڍs�ɔ�����idpExpress.h�쐬
 * - 2014/04/10 ���� �q��
 *   - Doxygen�R�����g�ǉ�
 *   - �R���X�g���N�^�E�֐��̐���
 * - 2014/04/25 ���� �q��
 *   - SSE�����ǉ�
 * - 2014/11/20 ���c�x
 *   - USE_ONLINE���s�v�������폜
 * - 2015/11/18 ���c�x
 *   - USE_BAYER_TRANSFORM��ǉ����CgetFrame�Ƀx�C���[�ϊ���ǉ�
 * - 2015/12/11 ���c�x 
 *   - getFrame, getFrameROI, getFrameStereo, getFrameStereoROI�̖߂�l��bool�ɕύX
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
/** �t���[�����[�g[fps](50, 60, 125, 250, 500, 1000����I���\) */
#define FRAME_PER_SEC			1000
/** �J�����w�b�hID(2��ڑ�����1�䂾���g�p����ꍇ, 0or1) */
#define USC_CAM_HEAD_ID			0
/** �摜�T�C�Y�� */
#define CAMERA_WIDTH				512
/** �摜�T�C�Y�c */
#define CAMERA_HEIGHT				512
/** �X�e���I�ڑ�����SSE��p���������������邩�ǂ���(1�͎g��, 0�͎g��Ȃ�) */
#define USE_SSE					0
/** getFrame()�֐��Ńx�C���[�ϊ����s�����ǂ��� (1�͎g��, 0�͎g��Ȃ�) */
#define USE_BAYER_TRANSFORM 0


//-----------------------------------------------------------------------------
// Class
//-----------------------------------------------------------------------------
/**
 * @class	idpExpress
 * @brief	Photron IDP-Express�摜�擾�p�N���X
 */
class idpExpress{

private:
	/** �ڑ����ꂽ�f�o�C�X�̌������� */
	PDC_DETECT_NUM_INFO DetectNumInfo;
	/** �f�o�C�X�ԍ� */
	unsigned long nDeviceNo;
	/** �f�o�C�X�̏�� */
	unsigned long nStatus;
	/** �q�f�o�C�X�̐� */
	unsigned long nCount;
	/** �q�f�o�C�X�ԍ� */
	unsigned long ChildNo[PDC_MAX_DEVICE];
	/** �擾�����摜�̔ԍ� */
	unsigned long nFrameNo;
	/** ��O�̉摜�̔ԍ� */
	unsigned long nOldFrameNo;
	/** �֐����۔���ϐ� */
	unsigned long nRet;
	/** �G���[�ԍ� */
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