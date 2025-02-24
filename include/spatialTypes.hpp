#ifndef SPATIALTYPES_HPP
#define SPATIALTYPES_HPP

#include <vector>
#include "eigen3/Eigen/Dense"

using RotMat = Eigen::Matrix<double, 3, 3>;

using Quatd = Eigen::Quaterniond;

using Quatf = Eigen::Quaternionf;

using sMat = Eigen::Matrix<double, 6,6>;

using sVec = Eigen::Matrix<double, 6,1>;

using Mat6 = Eigen::Matrix<double, 6,6>;

using Mat3 = Eigen::Matrix<double, 3,3>;

using Vec6 = Eigen::Matrix<double, 6,1>;

using Vec3 = Eigen::Matrix<double, 3,1>;

template <typename T>
using vectorAligned = std::vector<T, Eigen::aligned_allocator<T>>;

#endif