//-----------------------------------------------------------------------------
// Class for High-speed Projector
// -- Gaku Narita, 2015/11/06
// -- Toshiyuki Kato, 2016/06/29
// --                 2016/07/07 - Add DummyHighSpeedProjecter
//-----------------------------------------------------------------------------

#pragma once
#include <tchar.h>
#include <windows.h>
#include <assert.h>
#ifdef __cplusplus
#include <stdexcept>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <iostream>
#endif

#define PROJ_WIDTH 1024
#define PROJ_HEIGHT 768

#define LED_CTRL_STEP 20

#ifdef __cplusplus
namespace HighSpeedProjecter{
#endif
	
	// -------- libappinterface.h と一致させるべき部分
	struct HSP_Parameter{
		// SYSTEM_PARAM 関連
		ULONG	FrameRate;		// fps
		ULONG	ValidBits;		// 8: output 0:7, 7: output 1:7, 6: output 2:7, 5: output 3:7
		ULONG	MirrorMode;		// 0: enable 1: disable
		ULONG	CompData;		// 0: enable 1: disable
		ULONG	FlipMode;		// 0: enable 1: disable
		int		LEDAdjust[LED_CTRL_STEP];
		// PCIE_LED_ENABLE_DELAY
		ULONG	LEDCtrlDelay;
		// PCIE_FRAME_TRIG_DELAY
		ULONG	FrameTrigDelay;

		ULONG	NumOfDMABufferFrame;	// DMAのバッファーのフレーム数（libappinterface.h内のFRAME_BUF_MIN_NUMとFRAME_BUF_MAX_NUMの間に制約されている（このコメントを記載した時点では、100-1000の間））
	};
	// -------- libappinterface.h と一致させるべき部分 ここまで

#ifdef __cplusplus
	using Parameter = HSP_Parameter;
#endif
	
#ifdef __cplusplus
	namespace detail{
		extern "C"{
#endif
			struct HSP;
			struct HSP *HSP_create(void);
			BOOL HSP_connect(struct HSP *data, struct HSP_Parameter const *p);
			void HSP_disconnect(struct HSP *data);
			void HSP_error_close(struct HSP *data);
			void HSP_destroy(struct HSP *data);
			void HSP_send(struct HSP *data, void *img, unsigned int stride);
			BOOL HSP_get_buffer(struct HSP *data, void **pbuff, ULONG *psize);
			BOOL HSP_post_data(struct HSP *data);
#ifdef __cplusplus
		} // extern "C"
	} // namespace detail
#endif
    
    #define HSP_DEFAULT_PARAMETER_BRACE { \
		1000,															/* FrameRate */ \
		8,																/* ValidBits */ \
		1,																/* MirrorMode */ \
		0,																/* CompData */ \
		0,																/* FlipMode */ \
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	/* LEDAdjust */ \
		0,																/* LEDCtrlDelay */ \
		0,																/* FrameTrigDelay */ \
		100,															/* NumOfBufferFrame */ \
	}
    
	#ifdef __cplusplus
	constexpr
	#endif
	static const struct HSP_Parameter HSP_default_parameter = HSP_DEFAULT_PARAMETER_BRACE;
	#ifdef __cplusplus
	static const Parameter default_parameter = HSP_default_parameter;
	#endif

	#ifdef __cplusplus
	class HighSpeedProjecter{
	private:
		detail::HSP *data;

	public:
		HighSpeedProjecter()
			: data(detail::HSP_create())
		{
			if(data == nullptr)
			{
				throw std::bad_alloc();
			}
		}
		HighSpeedProjecter(HighSpeedProjecter const &right) = delete;
		HighSpeedProjecter(HighSpeedProjecter &&right){
			*this = std::move(right);
		}
		HighSpeedProjecter &operator=(HighSpeedProjecter const &right) = delete;
		HighSpeedProjecter &operator=(HighSpeedProjecter &&right){
			data = right.data;
			right.data = nullptr;
		}
		~HighSpeedProjecter(){
			detail::HSP_destroy(data);
		}
		bool connectProj(Parameter const &p = default_parameter){
			return detail::HSP_connect(data, &p) ? true : false;
		}
		void disconnectProj(){
			detail::HSP_disconnect(data);
		}
		void errorCloseProj(){
			detail::HSP_error_close(data);
		}
		void sendImage(void *img, unsigned int stride = 0u){
			detail::HSP_send(data, img, stride);
		}
	};
	class DummyHighSpeedProjecter{
	public:
		DummyHighSpeedProjecter(){
			std::cout << "DummyHighSpeedProjecter is being used" << std::endl;
		}
		DummyHighSpeedProjecter(DummyHighSpeedProjecter const &right) = delete;
		DummyHighSpeedProjecter(DummyHighSpeedProjecter &&right){
		}
		DummyHighSpeedProjecter &operator=(DummyHighSpeedProjecter const &right) = delete;
		DummyHighSpeedProjecter &operator=(DummyHighSpeedProjecter &&right){
		}
		~DummyHighSpeedProjecter(){
		}
		bool connectProj(Parameter const &p = default_parameter){
			return true;
		}
		void disconnectProj(){
		}
		void errorCloseProj(){
		}
		void sendImage(void *img){
			static std::uint8_t data[PROJ_WIDTH * PROJ_HEIGHT];
			std::uint8_t *ptr = static_cast<std::uint8_t *>(img);
			for(unsigned int i = 0; i < PROJ_WIDTH * PROJ_HEIGHT; ++i){
				data[i] = ptr[i];
			}
		}
		void sendImageRGBAToMono(void *img){
			static std::uint8_t data[PROJ_WIDTH * PROJ_HEIGHT];
			std::uint8_t *ptr = static_cast<std::uint8_t *>(img);
			for(unsigned int i = 0; i < PROJ_WIDTH * PROJ_HEIGHT; ++i){
				data[i] = ptr[i * 4];
			}
		}
	};
	class BMPHighSpeedProjecter{
	private:
		unsigned long long int frame_num = 0;
		unsigned int step;

	public:
		BMPHighSpeedProjecter(unsigned int step = 1000)
			: step(step)
		{
			std::cout << "BMPHighSpeedProjecter is being used" << std::endl;
		}
		BMPHighSpeedProjecter(BMPHighSpeedProjecter const &right) = delete;
		BMPHighSpeedProjecter(BMPHighSpeedProjecter &&right){
		}
		BMPHighSpeedProjecter &operator=(BMPHighSpeedProjecter const &right) = delete;
		BMPHighSpeedProjecter &operator=(BMPHighSpeedProjecter &&right){
		}
		~BMPHighSpeedProjecter(){
		}
		bool connectProj(Parameter const &p = default_parameter){
			return true;
		}
		void disconnectProj(){
		}
		void errorCloseProj(){
		}
	private:
		static void writeBMP(char const *const filename, std::uint32_t const width, std::uint32_t const height, void *ptr){
			HANDLE hFile = CreateFileA(filename, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
			if(hFile == INVALID_HANDLE_VALUE) return;
			std::uint32_t const pixel_num = width * height;
			std::uint32_t const filesize = 14 + 40 + pixel_num * 3;
			#define UINT16TO2BYTE(NUM) (std::uint16_t)(NUM) & 0xFF, ((std::uint16_t)(NUM) >> 8) & 0xFF
			#define UINT32TO4BYTE(NUM) (std::uint32_t)(NUM) & 0xFF, ((std::uint32_t)(NUM) >> 8) & 0xFF, ((std::uint32_t)(NUM) >> 16) & 0xFF, ((std::uint32_t)(NUM) >> 24) & 0xFF
			DWORD written;
			std::uint8_t const header[14] =
				{
					'B', 'M',
					UINT32TO4BYTE(filesize),
					UINT16TO2BYTE(0),
					UINT16TO2BYTE(0),
					UINT32TO4BYTE(14 + 40)
				};
			WriteFile(hFile, header, sizeof(header), &written, nullptr);
			std::uint8_t const infoheader[40] =
				{
					UINT32TO4BYTE(40),
					UINT32TO4BYTE(width),
					UINT32TO4BYTE(~height + 1),
					UINT16TO2BYTE(1),
					UINT16TO2BYTE(24),
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
				};
			WriteFile(hFile, infoheader, sizeof(infoheader), &written, nullptr);
			assert((width * 3) % 8 == 0);
			WriteFile(hFile, ptr, pixel_num * 3, &written, nullptr);
			CloseHandle(hFile);
			#undef UINT16TO2BYTE
			#undef UINT32TO4BYTE
		}
		void writeBMPFilename(void *ptr){
			unsigned long long int const img_num = frame_num / step;

			char str[120];
			std::snprintf(str, std::size(str), "%05llu.bmp", img_num);
			writeBMP(str, PROJ_WIDTH, PROJ_HEIGHT, ptr);
		}
		bool isWriteFrame(){
			return (frame_num++ % step) == 0;
		}
	public:
		void sendImage(void *img){
			if(!isWriteFrame()) return;
			auto data = std::make_unique<std::uint8_t[]>(PROJ_WIDTH * PROJ_HEIGHT * 3);
			for(unsigned int i = 0; i < PROJ_WIDTH * PROJ_HEIGHT * 3; ++i){
				data[i] = static_cast<std::uint8_t *>(img)[i / 3];
			}
			writeBMPFilename(data.get());
		}
		// 面倒くさいのでRだけ抜き出す
		void sendImageRGBAToMono(void *img){
			if(!isWriteFrame()) return;
			auto data = std::make_unique<std::uint8_t[]>(PROJ_WIDTH * PROJ_HEIGHT * 3);
			for(unsigned int i = 0; i < PROJ_WIDTH * PROJ_HEIGHT * 3; ++i){
				data[i] = static_cast<std::uint8_t *>(img)[i / 3 * 4];
			}
			writeBMPFilename(data.get());
		}
	};
	#endif

#ifdef __cplusplus
}	// namespace HighSpeedProjecter

using HighSpeedProj = HighSpeedProjecter::HighSpeedProjecter;
using DummyHighSpeedProj = HighSpeedProjecter::DummyHighSpeedProjecter;
using BMPHighSpeedProj = HighSpeedProjecter::BMPHighSpeedProjecter;
#endif
