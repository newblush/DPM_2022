#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <utility>
#include <fstream>
#include <cstdint>
#include <algorithm>
#include <memory>
#include "tracking_data.hpp"
#include "recognition_thread.hpp"
#include "tracking_thread.hpp"
#include "pnp_track_thread.hpp"
#include "draw_thread.hpp"
#include "draw_opengl_shade.hpp"
#include "ring_buffer.hpp"
#include "capture_thread.hpp"
#include "capture_class.hpp"
#include "model_io.hpp"
#include "load_camera_data.hpp"
#include "vector.hpp"
#include "make_point.hpp"
#include "recognizer.hpp"
#include "ObjData.h"

#include "HighSpeedProjector.h"
#include "ProjectorUtility.h"

#ifdef _MSC_VER
#if !_DEBUG
#pragma comment( lib, "opencv_world310" )
#pragma comment( lib, "fade2D_x64_v140_Release" )
#else
#pragma comment( lib, "opencv_world310d" )
#pragma comment( lib, "fade2D_x64_v140_Debug" )
#endif
#endif

//High-Speed Camera
#define USE_BASLER
#ifdef USE_BASLER
#include <HSC/baslerClass.hpp>
#ifdef _DEBUG
#pragma comment(lib, "BaslerLibd")
#else
#pragma comment(lib, "BaslerLib")
#endif
#endif

std::atomic< bool > g_please_change_objdata_index = false;

#define WITH_SAVE_THREAD 0

#if WITH_SAVE_THREAD
#  include "save_thread.hpp"
#endif

static
std::string to_local_path(std::string filepath)
{
#if _WIN32
    std::string::size_type pos = 0u;
    while (true)
    {
        pos = filepath.find('/', pos);
        if (pos == std::string::npos) break;
        filepath.replace(pos, 1, 1, '\\');
        pos += 1;
    }
#endif
    return std::move(filepath);
}

int main()
try
{
    using namespace std::string_literals;
    using meta::as_const;

    // データのロード
    // std::vector< float > point, dcm_normal, draw_point, draw_normal;
    // std::vector< unsigned int > index, num, draw_index;
    // cv::Mat cameraMatrix, cameraCoeffs;
    std::vector< DCMObject > objdata;
    // Data for 胸像
    /*
    {
        fv point, dcm_normal, draw_point, draw_normal;
        uiv index, num, draw_index;
        cv::Mat cameraMatrix, cameraCoeffs;
        auto const &to_data = R"(../../exp_data/)"s;
        //auto const &to_calibrate_data = R"(../../CalibrateCamera/CalibrateCamera/output_1484668423/)"s;
        auto const &to_calibrate_data = R"(../../CalibrateCamera/CalibrateCamera/output_1506943551/)"s;
        auto const &filename = to_local_path( to_data + R"(Lau_150_normalized.ply)"s );
        auto const &filename_num = to_local_path( to_data + R"(lau/150.ply.clu6.1476391063.dat)"s );
        auto const &filename_hires = to_local_path( to_data + R"(Lau_hires_normalized.ply)" );
        std::tie( point, index ) = load_ply( filename );
        num = load_num( filename_num );
        auto dual = make_dual( point, index );
        dcm_normal = calc_dcm_cluster_normal( std::move( std::get< 0 >( dual ) ), std::move( std::get< 1 >( dual ) ) );
        std::tie( draw_point, draw_index ) = load_ply( filename_hires );
        draw_normal = calc_normal( draw_point, draw_index );
        cameraMatrix = load_camera_matrix( to_local_path( to_calibrate_data + R"(cameraMatrix.dat)"s ) );
        cameraCoeffs = load_camera_coeffs( to_local_path( to_calibrate_data + R"(cameraCoeffs.dat)"s ) );
        objdata.emplace_back( point, dcm_normal, draw_point, draw_normal, index, num, draw_index, cameraMatrix, cameraCoeffs );
    }
    // */
    // Data for バニー
    // /*
    {
        fv point, dcm_normal, draw_point, draw_normal;
        uiv index, num, draw_index;
        cv::Mat cameraMatrix, cameraCoeffs;
        auto const& to_data = R"(../../exp_data/)"s;
        auto const& to_calibrate_data = R"(../../CalibrateCamera/CalibrateCamera/output_1659697933/)"s;
        auto const& filename = to_local_path(to_data + R"(stanford_bunny_LP.ply)"s);
        auto const& filename_num = to_local_path(to_data + R"(stanford_bunny_LP.ply.6.1487087494.clu)"s);
        auto const& filename_hires = to_local_path(to_data + R"(stanford_bunny.ply)");
        std::tie(point, index) = load_ply(filename);
        num = load_num(filename_num);
        auto dual = make_dual(point, index);
        dcm_normal = calc_dcm_cluster_normal(std::move(std::get< 0 >(dual)), std::move(std::get< 1 >(dual)));
        std::tie(draw_point, draw_index) = load_ply(filename_hires);
        draw_normal = calc_normal(draw_point, draw_index);
        cameraMatrix = load_camera_matrix(to_local_path(to_calibrate_data + R"(cameraMatrix.dat)"s));
        cameraCoeffs = load_camera_coeffs(to_local_path(to_calibrate_data + R"(cameraCoeffs.dat)"s));
        objdata.emplace_back(point, dcm_normal, draw_point, draw_normal, index, num, draw_index, cameraMatrix, cameraCoeffs);
    }
    // */
    ring_buffer< cv::Mat > rb(100);
    ////////
    // auto imgvec = load_images( to_local_path( R"(./data/image_C001H001S0001/image_C001H001S0001%06u.png)"s ) );
     //auto imgvec = load_images( to_local_path( R"(./data/bunny/img%03u.bmp)"s ) );
    //auto imgvec = load_images( to_local_path( R"(./data/bunny2/img%03u.jpg)"s ) );

    ////////

    //*
    //auto imgvec = load_images(to_local_path(R"(./rotation2/!save%04u.png)"s));
    capture_thread< basler_capture > ct(rb, 0u);

    //*
    HighSpeedProjector proj_V3;
    DYNAFLASH_PARAM param = getDefaultDynaParamRGB();
    param.dFrameRate = 940.0;
    param.nMirrorMode = 1;
    param.nFlipMode = 0;
    printDynaParam(param);

    proj_V3.connect(0);
    proj_V3.setParam(param);
    proj_V3.start();
    // */
    
    //////////////////////////////////
    /*
    basler cam;

    int width = 720;
    int height = 540;
    float  fps = 250.0f;
    //float gain = 1.0f;

    auto img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));

    cam.connect(1);
    cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
    cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
    cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
    cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
    cam.setParam(paramTypeBasler::Param::ExposureTime, 3950.0f);
    cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
    //cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
    cam.setParam(paramTypeBasler::GrabStrategy::OneByOne);
    //cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
    //cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);
    cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
    cam.parameter_all_print();

    cam.start();

    bool flag = true;
    std::thread thr([&] {
        while (flag) {
            cam.captureFrame(img.data);
        }
    });
    
    while (1) {
        cv::imshow("img", img);
        int key = cv::waitKey(1);
        if (key == 'q')break;
    }

    cam.stop();
    cam.disconnect();
    
    //*/
    //////////////////////////////////
    
    
    //ximea_xiq_capture cap( 500.0f );
    //capture_thread< std::decay_t< decltype( cap ) > > ct( rb, std::move( cap ), 0u );
    ////////
    //std::cout << "image width: " << ct.width() << ", height: " << ct.height() << std::endl;

    std::vector< tracking_data > tds(std::size(objdata));
    for (std::size_t i = 0u; i < std::size(tds); ++i)
    {
        tds[i].point.emplace_back(std::make_unique< point_tracking_data[] >(std::size(std::get< DPOINT >(objdata[i])) / 3));
        tds[i].point_size.push_back(static_cast<unsigned int>(std::size(std::get< DPOINT >(objdata[i])) / 3));
    }

    using uivv = std::vector< uiv >;
    using uivvv = std::vector< uivv >;
    using fvv = std::vector< fv >;
    using fvvv = std::vector< fvv >;

    uivvv num_vector_vector, index_vector_vector, draw_index_vector_vector;
    fvvv point_vector_vector, dcm_normal_vector_vector, draw_point_vector_vector, draw_normal_vector_vector;
    for (auto const& o : objdata)
    {
        std::vector< std::vector< unsigned int > > num_vector{ std::get< NUM >(o) };
        std::vector< std::vector< float > > point_vector{ std::get< DPOINT >(o) };
        std::vector< std::vector< unsigned int > > index_vector{ std::get< INDEX >(o) };
        std::vector< std::vector< float > > dcm_normal_vector{ std::get< DCM_NORMAL >(o) };
        std::vector< std::vector< float > > draw_point_vector{ std::get< DRAW_POINT >(o) };
        std::vector< std::vector< unsigned int > > draw_index_vector{ std::get< DRAW_INDEX >(o) };
        std::vector< std::vector< float > > draw_normal_vector{ std::get< DRAW_NORMAL >(o) };

        num_vector_vector.emplace_back(num_vector);
        point_vector_vector.emplace_back(point_vector);
        index_vector_vector.emplace_back(index_vector);
        dcm_normal_vector_vector.emplace_back(dcm_normal_vector);
        draw_point_vector_vector.emplace_back(draw_point_vector);
        draw_index_vector_vector.emplace_back(draw_index_vector);
        draw_normal_vector_vector.emplace_back(draw_normal_vector);
    }

    ct.run();


    std::atomic< std::size_t > objdata_index = ~0u;

    draw_opengl_shade dot(proj_V3,tds, as_const(rb), point_vector_vector, draw_point_vector_vector, draw_index_vector_vector, draw_normal_vector_vector, std::get< CAMERA_MATRIX >(objdata[0]), std::get< CAMERA_COEFFS >(objdata[0]), objdata_index);
    dot.run();

    while (true)
    {
        auto const i = (objdata_index.load(std::memory_order::memory_order_acquire) + 1) % std::size(objdata);
        objdata_index.store(i, std::memory_order::memory_order_release);
        auto const& o = objdata[i];

        recognition::larger_triangle_recognizer ltr({ std::make_tuple(0u, std::get< INDEX >(o), std::get< NUM >(o)) });

        recognition_thread< recognition::larger_triangle_recognizer > rt(tds[i], as_const(rb), ltr);
        tracking_thread tt(tds[i], as_const(rb), num_vector_vector[i]);
        pnp_track_thread ptt(tds[i], as_const(rb), num_vector_vector[i], point_vector_vector[i], dcm_normal_vector_vector[i], std::get< CAMERA_MATRIX >(o), std::get< CAMERA_COEFFS >(o));
        draw_thread dt(tds[i], as_const(rb), point_vector_vector[i], std::get< CAMERA_MATRIX >(o), std::get< CAMERA_COEFFS >(o));
#if WITH_SAVE_THREAD
        save_thread st(td, as_const(rb), { std::make_tuple(num, index) }, { point }, cameraMatrix, cameraCoeffs);
#endif

        rt.run();
        tt.run();
        ptt.run();
        dt.run();
        /*
        while (true)
        {
            if(ptt.solve_pnp_point_img.rows == 0)
            Sleep(1);
            else
            break;
        }
        cv::Mat Homography = (cv::Mat_<double>(3, 3) << 1.1790856, 0.052938782, 55.723301,
        -0.058890902, 1.1125003, 243.97942,
        4.1042145e-05, -6.6375411e-05, 1
            );
            cv::Mat img2(768, 1024, CV_8UC3);*/
        while (true)
        {
        /*
        char c;
            if(ptt.flag)
            {
            cv::warpPerspective(ptt.solve_pnp_point_img.clone(), img2, Homography, cv::Size(1024, 768));
            //cv::flip(img2, img2, -1);

            cv::imshow("cv", img2);
            c = cv::waitKey(1);
        //proj_V3.sendImage(img2.data);
        }
            if (g_please_change_objdata_index.load(std::memory_order::memory_order_acquire))
            {
                g_please_change_objdata_index.store(false, std::memory_order::memory_order_release);
                break;
            }
            */
            char c;
            std::cin >> c;
            if( c == 's' )
            {
                std::cout << "Stop!" << std::endl;
                break;
            }
            /*
            #define TOGGLE( v, name ) if( v.is_running() ){ std::cout << name " stop" << std::endl; v.stop(); } else { std::cout << name " start" << std::endl; v.run(); } break
            switch( c )
            {
            case 'c': TOGGLE( ct, "Capture Thread" );
            case 'r': TOGGLE( rt, "Recognition Thread" );
            case 't': TOGGLE( tt, "Tracking Thread" );
            case 'p': TOGGLE( ptt, "PnP Tracking Thread" );
            case 'd': TOGGLE( dt, "Draw Thread" );
            #undef TOGGLE
#if WITH_SAVE_THREAD
            case 'o': st.run(); break;
#endif
            }
            */
        }
        dt.stop();
        ptt.stop();
        tt.stop();
        rt.stop();

        proj_V3.stop();
        proj_V3.disconnect();

#if WITH_SAVE_THREAD
        st.stop();
#endif
        }
    dot.stop();
    ct.stop();

    }
catch (std::exception& e)
{
    std::cerr << e.what() << std::endl;
    char c;
    std::cin >> c;
}
