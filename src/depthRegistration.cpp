#include "depthRegistration.h"

DepthRegistration::DepthRegistration(){

}

DepthRegistration::~DepthRegistration(){

}

void DepthRegistration::createLookup(){
    const double fx = 1.0 / cameraMatrixRegistered.at<double>(0, 0);
    const double fy = 1.0 / cameraMatrixRegistered.at<double>(1, 1);
    const double cx = cameraMatrixRegistered.at<double>(0, 2);
    const double cy = cameraMatrixRegistered.at<double>(1, 2);
    double *it;
    lookupY = cv::Mat(1, sizeRegistered.height, CV_64F);
    it = lookupY.ptr<double>();
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }
    lookupX = cv::Mat(1, sizeRegistered.width, CV_64F);
    it = lookupX.ptr<double>();
    for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
}

bool DepthRegistration::init(){
    createLookup();
    proj(0, 0) = rotation.at<double>(0, 0);
    proj(0, 1) = rotation.at<double>(0, 1);
    proj(0, 2) = rotation.at<double>(0, 2);
    proj(0, 3) = translation.at<double>(0, 0);
    proj(1, 0) = rotation.at<double>(1, 0);
    proj(1, 1) = rotation.at<double>(1, 1);
    proj(1, 2) = rotation.at<double>(1, 2);
    proj(1, 3) = translation.at<double>(1, 0);
    proj(2, 0) = rotation.at<double>(2, 0);
    proj(2, 1) = rotation.at<double>(2, 1);
    proj(2, 2) = rotation.at<double>(2, 2);
    proj(2, 3) = translation.at<double>(2, 0);
    proj(3, 0) = 0;
    proj(3, 1) = 0;
    proj(3, 2) = 0;
    proj(3, 3) = 1;
    fx = cameraMatrixRegistered.at<double>(0, 0);
    fy = cameraMatrixRegistered.at<double>(1, 1);
    cx = cameraMatrixRegistered.at<double>(0, 2) + 0.5;
    cy = cameraMatrixRegistered.at<double>(1, 2) + 0.5;
    return true;
}

bool DepthRegistration::init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
                             const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
                             const float zNear, const float zFar)
{
  this->cameraMatrixRegistered = cameraMatrixRegistered;
  this->cameraMatrixDepth = cameraMatrixDepth;
  this->rotation = rotation;
  this->translation = translation;
  this->sizeRegistered = sizeRegistered;
  this->sizeDepth = sizeDepth;
  this->zNear = zNear;
  this->zFar = zFar;

  cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixRegistered, sizeRegistered, CV_32FC1, mapX, mapY);

  return init();
}

uint16_t DepthRegistration::interpolate(const cv::Mat &in, const float &x, const float &y) const{
    const int xL = (int)floor(x);
    const int xH = (int)ceil(x);
    const int yL = (int)floor(y);
    const int yH = (int)ceil(y);

    if(xL < 0 || yL < 0 || xH >= in.cols || yH >= in.rows)
    {
      return 0;
    }

    const u_int16_t pLT = in.at<u_int16_t>(yL, xL);
    const u_int16_t pRT = in.at<u_int16_t>(yL, xH);
    const u_int16_t pLB = in.at<u_int16_t>(yH, xL);
    const u_int16_t pRB = in.at<u_int16_t>(yH, xH);
    int vLT = pLT > 0;
    int vRT = pRT > 0;
    int vLB = pLB > 0;
    int vRB = pRB > 0;
    int count = vLT + vRT + vLB + vRB;

    if(count < 3)
    {
      return 0;
    }

    const uint16_t avg = (pLT + pRT + pLB + pRB) / count;
    const uint16_t thres = 0.01 * avg;
    vLT = abs(pLT - avg) < thres;
    vRT = abs(pRT - avg) < thres;
    vLB = abs(pLB - avg) < thres;
    vRB = abs(pRB - avg) < thres;
    count = vLT + vRT + vLB + vRB;

    if(count < 3)
    {
      return 0;
    }

    double distXL = x - xL;
    double distXH = 1.0 - distXL;
    double distYL = y - yL;
    double distYH = 1.0 - distYL;
    distXL *= distXL;
    distXH *= distXH;
    distYL *= distYL;
    distYH *= distYH;
    const double tmp = sqrt(2.0);
    const double fLT = vLT ? tmp - sqrt(distXL + distYL) : 0;
    const double fRT = vRT ? tmp - sqrt(distXH + distYL) : 0;
    const double fLB = vLB ? tmp - sqrt(distXL + distYH) : 0;
    const double fRB = vRB ? tmp - sqrt(distXH + distYH) : 0;
    const double sum = fLT + fRT + fLB + fRB;

    return ((pLT * fLT +  pRT * fRT + pLB * fLB + pRB * fRB) / sum) + 0.5;
}

void DepthRegistration::remapDepth(const cv::Mat &depth, cv::Mat &scaled) const{
    scaled.create(sizeRegistered, CV_16U);
    #pragma omp parallel for
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
    {
      u_int16_t *itO = scaled.ptr<u_int16_t>(r);
      const float *itX = mapX.ptr<float>(r);
      const float *itY = mapY.ptr<float>(r);
      for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itO, ++itX, ++itY)
      {
        *itO = interpolate(depth, *itX, *itY);
      }
    }
}

void DepthRegistration::projectDepth(const cv::Mat &scaled, cv::Mat &registered) const{
    registered = cv::Mat::zeros(sizeRegistered, CV_16U);
    #pragma omp parallel for
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
    {
      const u_int16_t *itD = scaled.ptr<u_int16_t>(r);
      const double y = lookupY.at<double>(0, r);
      const double *itX = lookupX.ptr<double>();

      for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itD, ++itX)
      {
        const double depthValue = *itD / 1000.0;

        if(depthValue < zNear || depthValue > zFar)
        {
          continue;
        }

        Eigen::Vector4d pointD(*itX * depthValue, y * depthValue, depthValue, 1);
        Eigen::Vector4d pointP = proj * pointD;

        const double z = pointP[2];

        const double invZ = 1 / z;
        const int xP = (fx * pointP[0]) * invZ + cx;
        const int yP = (fy * pointP[1]) * invZ + cy;

        if(xP >= 0 && xP < sizeRegistered.width && yP >= 0 && yP < sizeRegistered.height)
        {
          const u_int16_t z16 = z * 1000;
          u_int16_t &zReg = registered.at<u_int16_t>(yP, xP);
          if(zReg == 0 || z16 < zReg)
          {
            zReg = z16;
          }
        }
      }
    }
}

bool DepthRegistration::registerDepth(const cv::Mat &depth, cv::Mat &registered){
    cv::Mat scaled;
    remapDepth(depth, scaled);
    projectDepth(scaled, registered);
    return true;
}

