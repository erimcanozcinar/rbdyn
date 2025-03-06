#ifndef ORIENTTOOLS_HPP
#define ORIENTTOOLS_HPP

#include <cmath>
#include <iostream>
#include "spatialTypes.hpp"

namespace ori {

    enum class CoordinateAxis { X, Y, Z, None };

    /* Converts CoordinateAxis into string */
    std::string toString(CoordinateAxis axis);

    /* Constructs 3x3 rotation matrix from rotation axis and angle */
    RotMat coordinateRotation(CoordinateAxis axis, double theta);

    /* Converts quaternion into 3x3 rotation matrix */
    RotMat quat2RotMat(const Quatd& quat);

    /* Converts 3x3 vector into skew symmetric matrix */
    Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d& v);

    Eigen::Vector3d skewMatToVector(const Eigen::Matrix3d& m);

    /* Converts skew symmetric matrix into 3x3 vector */
    Eigen::Vector3d skewMatToVec(const Eigen::Matrix3d& m);

}

#endif