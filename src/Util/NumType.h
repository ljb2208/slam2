
#pragma once

#include "Eigen/Core"
#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"

typedef Eigen::Matrix<double,Eigen::Dynamic,1> VecX;
typedef Eigen::Matrix<double,3,3> Mat33;

typedef Eigen::Matrix<double,3,1> Vec3d;

typedef Eigen::Matrix<float,3,1> Vec3f;
typedef Eigen::Matrix<unsigned char,3,1> Vec3b;

typedef Sophus::SE3d SE3;
typedef Sophus::Sim3d Sim3;
typedef Sophus::SO3d SO3;