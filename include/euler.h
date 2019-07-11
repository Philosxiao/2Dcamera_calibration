#ifndef __EULER__
#define __EULER__

#include <iostream>
#include <string>
#include "Eigen/Dense"
#include <cmath>
#include <unordered_map>

const float _FLOAT_EPS =1e-7;
const float _EPS4 =_FLOAT_EPS * 4.0;

class Euler {
public:
    Euler();
    ~Euler();
    Eigen::Matrix3f euler2mat(float ai, float aj, float ak, std::string axes="sxyz");
    Eigen::Vector3f mat2euler(Eigen::Matrix3f &M, std::string axes="sxyz");
    Eigen::Quaternionf euler2quat(float ai, float aj, float ak, std::string axes="sxyz");
    Eigen::Vector3f quat2euler(Eigen::Quaternionf q,std::string axes="sxyz");
private:
    Eigen::Vector4i axes2vector(std::string axes);

};

#endif
