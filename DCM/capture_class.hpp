#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>

class video_capture
{
public:
    video_capture()
        : cap(0)
    {
        cap >> tmp;
    }
    void capture(cv::Mat& img)
    {
        cap >> tmp;
        cv::cvtColor(tmp, img, cv::COLOR_RGB2GRAY);
    }

    unsigned int width() const
    {
        return tmp.cols;
    }
    unsigned int height() const
    {
        return tmp.rows;
    }

private:
    cv::VideoCapture cap;
    cv::Mat tmp;
};

class cv_mat_capture
{
public:
    // vec.empty() must be false
    cv_mat_capture(std::vector< cv::Mat > _vec)
        : index(0u)
        , vec(std::move(_vec))
        , w(vec.front().cols)
        , h(vec.back().rows)
    {
#if _DEBUG
        for (auto&& m : vec)
        {
            if (w != m.cols || h != m.rows)
            {
                throw std::exception("error");
            }
        }
#endif
    }
    void capture(cv::Mat& img)
    {
        img = vec[index++ % vec.size()].clone();
    }

    unsigned int width() const
    {
        return w;
    }
    unsigned int height() const
    {
        return h;
    }

private:
    std::size_t index;
    std::vector< cv::Mat > vec;
    unsigned int const w;
    unsigned int const h;
};

//High-Speed Camera
#define USE_BASLER
#ifdef USE_BASLER
#include <HSC/baslerClass.hpp>
#include <windows.h>
#ifdef _DEBUG
#pragma comment(lib, "BaslerLibd")
#else
#pragma comment(lib, "BaslerLib")
#endif
#endif
class basler_capture
{
public:
    basler_capture():camera() {

        //basler camera;
        //cam = new basler();

        int width = 720;
        int height = 540;
        float gain = 1.0f;
        float exposuretime = 3950.0f;

        h = height;
        w = width;

        auto camera_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));

        camera.connect(0);
        camera.setParam(paramTypeCamera::paramInt::WIDTH, width);
        camera.setParam(paramTypeCamera::paramInt::HEIGHT, height);
        camera.setParam(paramTypeCamera::paramFloat::FPS, 250.0f);
        camera.setParam(paramTypeCamera::paramFloat::GAIN, gain);
        camera.setParam(paramTypeBasler::Param::ExposureTime, 3950.0f);
        camera.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
        //camera.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
        camera.setParam(paramTypeBasler::GrabStrategy::OneByOne);
        //camera.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
        //camera.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);
        camera.setParam(paramTypeBasler::CaptureType::MonocroGrab);
        camera.parameter_all_print();
        camera.start();
        //cam = &camera;
     
    }
    /*
    basler_capture(basler_capture const&) = delete;
    basler_capture(basler_capture&& r)
        : w(r.w), h(r.h)
        , cam(r.cam)
    {
        r.cam = nullptr;
    }
    ~basler_capture() {
        cam->stop();
        cam->disconnect();
    }
    basler_capture& operator=(basler_capture&) = delete;
    basler_capture& operator=(basler_capture&& r)
    {
        cam = r.cam;
        w = r.w; h = r.h;
        r.cam = nullptr;
    }*/
    void capture(cv::Mat& img)
    {
        img.create(h, w, CV_8UC1);
        //cam->start();
        camera.captureFrame(img.data);
    }
    unsigned int width() const
    {
        return w;
    }
    unsigned int height() const
    {
        return h;
    }
private:
    unsigned int w = 0u, h = 0u;
    basler camera;
};

#if 0
// #if __has_include( "PDCLIB.h" )

class idp_express_capture
{

};

#endif
