#ifndef __DEPTHREGISTRATION__
#define __DEPTHREGISTRATION__
#include <Eigen/Geometry>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
class DepthRegistration {
private:
    cv::Mat cameraMatrixRegistered, cameraMatrixDepth, rotation, translation, mapX, mapY;
    cv::Size sizeRegistered, sizeDepth;
    float zNear, zFar;
    cv::Mat lookupX, lookupY;
    Eigen::Matrix4d proj;
    double fx, fy, cx, cy;
public:

    DepthRegistration();
    ~DepthRegistration();
    bool init();
    bool init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
              const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
              const float zNear = 0.5f, const float zFar = 12.0f);
    bool registerDepth(const cv::Mat &depth, cv::Mat &registered);

private:
    void createLookup();

    u_int16_t interpolate(const cv::Mat &in, const float &x, const float &y) const;

    void remapDepth(const cv::Mat &depth, cv::Mat &scaled) const;
    void projectDepth(const cv::Mat &scaled, cv::Mat &registered) const;
};
#endif
