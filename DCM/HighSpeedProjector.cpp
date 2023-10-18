#include "HighSpeedProjector.h"

HighSpeedProjector::HighSpeedProjector() {
	if (!SetProcessWorkingSetSizeEx(
		GetCurrentProcess(),
		(2000UL * 1024 * 1024),  // dwMinimumWorkingSetSize
		(3000UL * 1024 * 1024),  // dwMinimumWorkingSetSize
		QUOTA_LIMITS_HARDWS_MIN_ENABLE)) {
		throw std::runtime_error("Failed SetProcessWorkingSetSize");
	}

	// create an instance of DynaFlash
	pDynaFlash = CreateDynaFlash();
	if (pDynaFlash == nullptr) {
		throw std::runtime_error("pDynaFlash is nullptr");
	}
}

HighSpeedProjector::~HighSpeedProjector() {
	this->destruct();
}

void HighSpeedProjector::destruct() {
	std::cout << "destruct DynaFlash" << std::endl;
	if (pDynaFlash) {
		this->stop();
		this->disconnect();
		ReleaseDynaFlash(&pDynaFlash);
		pDynaFlash = nullptr;
		dynaFrashType = NULL;
	}
}

void HighSpeedProjector::connect(int deviceId) {
	if (pDynaFlash == nullptr) {
		throw std::runtime_error("pDynaFlash is nullptr");
	}

	if (pDynaFlash->Connect(deviceId) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to Connect.");
	}
	this->deviceId = deviceId;

	if (pDynaFlash->Reset() != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to Reset.");
	}

	unsigned long nVersion;
	pDynaFlash->GetDynaFlashType(&nVersion);
	this->dynaFrashType = nVersion;
	std::cout << "DynaFlash  : 0x" << std::hex << nVersion << std::dec << std::endl;
	// printVersion();
}

void HighSpeedProjector::start() {
	if (pDynaFlash->Start() != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to Start");
	}
}

void HighSpeedProjector::stop() {
	pDynaFlash->Stop();
	pDynaFlash->ReleaseFrameBuffer();
	pDynaFlash->Float(0);  // Float the mirror device
}

void HighSpeedProjector::disconnect() {
	if (pDynaFlash) {
		if (pDynaFlash->Disconnect() != STATUS_SUCCESSFUL) {
			throw std::runtime_error("Failed to Disconnect.");
		}
	}
}

int HighSpeedProjector::setImage(const void *data, int frameCnt) {
	pDynaFlash->GetStatus(&stDynaFlashStatus);
	if ((stDynaFlashStatus.InputFrames - stDynaFlashStatus.OutputFrames) > this->frameBufferNum) {
		return 0;
	}

	if (pDynaFlash->GetFrameBuffer(&pBuf, &nFrameCnt) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to GetFrameBuffer.");
	}
	memcpy(pBuf, data, this->frameSize * frameCnt);
	pBuf += this->frameSize * frameCnt;
	return nFrameCnt;
}

int HighSpeedProjector::sendImage(const void *data) {
	pDynaFlash->GetStatus(&stDynaFlashStatus);
	if ((stDynaFlashStatus.InputFrames - stDynaFlashStatus.OutputFrames) > this->frameBufferNum) {
		return 0;
	}

	if (pDynaFlash->GetFrameBuffer(&pBuf, &nFrameCnt) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to GetFrameBuffer.");
	}

	if ((pBuf != nullptr) && (nFrameCnt != 0)) {
		memcpy(pBuf, data, this->frameSize);
		if (pDynaFlash->PostFrameBuffer(1) != STATUS_SUCCESSFUL) {
			throw std::runtime_error("Failed to PostFrameBuffer.");
		}
		return 1;
	}
	return 0;
}

int HighSpeedProjector::sendImage(void) {
	if (pDynaFlash->GetFrameBuffer(&pBuf, &nFrameCnt) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to GetFrameBuffer.");
	}

	if ((pBuf != nullptr) && (nFrameCnt != 0)) {
		if (pDynaFlash->PostFrameBuffer(1) != STATUS_SUCCESSFUL) {
			throw std::runtime_error("Failed to PostFrameBuffer.");
		}
		return 1;
	}
	return 0;
}

int HighSpeedProjector::setFrameBufferNum(int bufferNum) {
	this->frameBufferNum = bufferNum;
	return this->frameBufferNum;
}

int HighSpeedProjector::getFrameBufferNum() {
	return this->frameBufferNum;
}

int HighSpeedProjector::getFrameSize() {
	return this->frameSize;
}

int HighSpeedProjector::getDeviceId() {
	return this->deviceId;
}

void HighSpeedProjector::setParam(DYNAFLASH_PARAM param) {
	this->checkParam(param);
	this->frameMode = param.nBinaryMode;
	switch (this->frameMode) {
	case FRAME_MODE_BINARY:
		this->frameSize = FRAME_BUF_SIZE_BINARY;
		break;
	case FRAME_MODE_GRAY:
		this->frameSize = FRAME_BUF_SIZE_8BIT;
		break;
	case FRAME_MODE_RGB:
		this->frameSize = FRAME_BUF_SIZE_24BIT;
		break;
	case FRAME_MODE_RGBW:
		this->frameSize = FRAME_BUF_SIZE_32BIT;
		break;
	default:
		throw std::runtime_error("frameMode is invalid.");
	}

	if (pDynaFlash->SetParam(&param) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to SetParam.");
	}

	/*
	if (pDynaFlash->SetIlluminance(HIGH_MODE) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to SetIlluminance.");
	}
	*/
	if (pDynaFlash->AllocFrameBuffer(this->frameBufferNum) != STATUS_SUCCESSFUL) {
		throw std::runtime_error("Failed to AllocFrameBuffer.");
	}
}

int HighSpeedProjector::checkParam(DYNAFLASH_PARAM param) {
	if (dynaFrashType == NULL) {
		throw std::runtime_error("Connection Error.");
	} else if (dynaFrashType == DYNA_TYPE_1) {
		if (param.nBinaryMode != FRAME_MODE_BINARY && param.nBinaryMode != FRAME_MODE_GRAY) {
			throw std::runtime_error("nBinaryMode is invalid.");
		}
	} else if (dynaFrashType == DYNA_TYPE_2) {
		if (param.nBinaryMode != FRAME_MODE_BINARY && param.nBinaryMode != FRAME_MODE_GRAY) {
			throw std::runtime_error("nBinaryMode is invalid.");
		}
	} else if (dynaFrashType == DYNA_TYPE_3) {
		if (param.nBinaryMode != FRAME_MODE_BINARY && param.nBinaryMode != FRAME_MODE_GRAY && param.nBinaryMode != FRAME_MODE_RGB) {
			throw std::runtime_error("nBinaryMode is invalid.");
		}
		double sum = param.dRProportion + param.dGProportion + param.dBProportion + param.dWProportion;
		if (sum != 100.0 || param.dWProportion != 0.0) {
			throw std::runtime_error("Proportion is invalid.");
		}
	} else if (dynaFrashType == DYNA_TYPE_4) {
		if (param.nBinaryMode != FRAME_MODE_BINARY && param.nBinaryMode != FRAME_MODE_GRAY && param.nBinaryMode != FRAME_MODE_RGB && param.nBinaryMode != FRAME_MODE_RGBW) {
			throw std::runtime_error("nBinaryMode is invalid.");
		}
		double sum = param.dRProportion + param.dGProportion + param.dBProportion + param.dWProportion;
		if (sum != 100.0) {
			throw std::runtime_error("The proportion is invalid.");
		}
	}
	return 0;
}

void HighSpeedProjector::printVersion() {
	char DriverVersion[40];
	unsigned long nVersion;

	// driver version
	pDynaFlash->GetDriverVersion(DriverVersion);
	std::cout << "DRIVER Ver : " << DriverVersion << std::endl;

	// dll version
	pDynaFlash->GetDLLVersion(&nVersion);
	std::cout << "DLL Ver    : " << nVersion << std::endl;

	// HW version
	pDynaFlash->GetHWVersion(&nVersion);
	std::cout << "HW Ver     : " << nVersion << std::endl;

	// DynaType
	pDynaFlash->GetDynaFlashType(&nVersion);
	std::cout << "DynaFlash  : 0x" << std::hex << nVersion << std::dec << std::endl;
}

DYNAFLASH_STATUS HighSpeedProjector::getStatus() {
	pDynaFlash->GetStatus(&stDynaFlashStatus);
	return stDynaFlashStatus;
}
