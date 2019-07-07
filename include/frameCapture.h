#ifndef __FRAMECAPTURE__
#define __FRAMECAPTURE__

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/libfreenect2.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <sys/stat.h>
#include "depthRegistration.h"

#include <unistd.h>
#include "filter.h"
#include "util.h"
#include "Timer.h"
//#include <librealsense2/rs.hpp>

#define K2_DEFAULT_NS          "kinect2"
#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"


typedef struct rs2_intrinsics
{
    int           width;     /**< Width of the image in pixels */
    int           height;    /**< Height of the image in pixels */
    double         ppx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    double         ppy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    double         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
    double         fy;        /**< Focal length of the image plane, as a multiple of pixel height */
    double         coeffs[5]; /**< Distortion coefficients, order: k1, k2, p1, p2, k3 */
} rs2_intrinsics;

//using namespace cv;

class FrameCapture {
private:
    cv::Size sizeColor, sizeIr;
    cv::Mat cameraMatrixColor, distortionColor, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
    cv::Mat rotation, translation;
    cv::Mat map1Color, map2Color,map1Ir,map2Ir;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *device;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::PacketPipeline *packetPipeline;
    libfreenect2::Registration *registration;
    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;

    DepthRegistration *depthRegHighRes;

    string sensor;
    double depthShift;
    bool running;
    int kinect_id;
    int capture_frame_A;
    int capture_frame_B;
    string calib_params_path_A;
    string calib_params_path_B;
    string config_path;
    string calib_path;
    cv::Mat rgbmat,depthmat,depth_raw;
    bool use_default_params;

public:
    FrameCapture();
    ~FrameCapture();
    bool start();
    void stop();
    bool initialize();
    bool initPipeline(const std::string &method, const int32_t device);
    bool initDevice(std::string &sensor);
    bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;
    bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;
    bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const;
    void initCalibration(const std::string &calib_path, const std::string &sensor);
    void processIrDepth(const cv::Mat &depth);
    bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames);
    void receiveRGBD();
    void processColor(const cv::Mat &color);
    void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);
    bool initRegistration( const double maxDepth);
    int getCaptureFrameA(){return capture_frame_A;}
    int getCaptureFrameB(){return capture_frame_B;}
    void capture();
    void setKinectId(int idx);
    cv::Mat depthRefinement(cv::Mat &depth, cv::Mat &rgb);
    cv::Mat getRgbMat(){return rgbmat;}
    cv::Mat getDepthMat(){return depthmat;}
    cv::Mat getDepthRaw(){return depth_raw;}
    void transToIdKinect(int idx);
    void default_receiveRGBD();

    rs2_intrinsics get_color_int(){
        rs2_intrinsics cam_int;
        cam_int.fx = cameraMatrixColor.at<double>(0, 0);
        cam_int.fy = cameraMatrixColor.at<double>(1, 1);
        cam_int.ppx = cameraMatrixColor.at<double>(0, 2);
        cam_int.ppy = cameraMatrixColor.at<double>(1, 2);

        return cam_int;
    }
    
};

#endif
