#ifndef SPATIAL_HPP
#define SPATIAL_HPP

#include "orientTools.hpp"

namespace spatial {
using namespace ori;
enum class JointType { Prismatic, Revolute, Spherical, Floating, Fixed };

/* Converts JointType into string */
std::string toString(JointType joint) {
    switch (joint) {
        case JointType::Prismatic: return "Prismatic";
        case JointType::Revolute: return "Revolute";
        case JointType::Spherical: return "Spherical";
        case JointType::Floating: return "Floating";
        case JointType::Fixed: return "Fixed";
        default: return "Unknown";
    }
}

/* Constructs Plücker coordinate transformation matrix for spesific axis of rotation
(translation part is zero) */
sMat spatialRotation(CoordinateAxis axis, double theta) {
    RotMat R = coordinateRotation(axis, theta);
    sMat X = sMat::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    return X;
}

sMat spatialTranslation(const Vec3 &r) {
    sMat X = sMat::Zero();
    X.template topLeftCorner<3, 3>() = RotMat::Zero();
    X.template bottomRightCorner<3, 3>() = RotMat::Zero();
    X.template bottomLeftCorner<3, 3>() = -vectorToSkewMat(r);
    return X;
}

/* Constructs a Plücker coordinate transformation matrix (6x6) */
auto spatialTransform(const RotMat& R, const Vec3& r) {
    sMat X = sMat::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    X.template bottomLeftCorner<3, 3>() = -R * vectorToSkewMat(r);
    return X;
}

/* Constructs a Plücker coordinate transformation matrix for specific joint type */
sMat jointSpatialTransform(JointType joint, CoordinateAxis axis, double q) {
    sMat X = sMat::Zero();
    RotMat R_float = RotMat::Zero();
    if (joint == JointType::Revolute) {
        X = spatialRotation(axis, q);
    } else if (joint == JointType::Prismatic) {
        Eigen::Vector3d v(0, 0, 0);
        if (axis == CoordinateAxis::X)
        v(0) = q;
        else if (axis == CoordinateAxis::Y)
        v(1) = q;
        else if (axis == CoordinateAxis::Z)
        v(2) = q;

        X = spatialTransform(RotMat::Identity(), v);
    } else if (joint == JointType::Fixed) {
        X = sMat::Identity();
    } else {
        throw std::runtime_error("Unknown joint xform\n");
    }
    return X;
}

/* Constructs joint motion subspace vector */
Vec6 jointMotionSubspace(JointType joint, CoordinateAxis axis) {
    Eigen::Vector3d v(0, 0, 0);
    Vec6 phi = Vec6::Zero();
    if (axis == CoordinateAxis::X)
        v(0) = 1;
    else if (axis == CoordinateAxis::Y)
        v(1) = 1;
    else
        v(2) = 1;

    if (joint == JointType::Prismatic)
        phi.template bottomLeftCorner<3, 1>() = v;
    else if (joint == JointType::Revolute)
        phi.template topLeftCorner<3, 1>() = v;
    else if (joint == JointType::Floating)
        phi = Vec6::Ones();
    else if (joint == JointType::Fixed)
        phi = Vec6::Zero();
    else
        throw std::runtime_error("Unknown motion subspace");

    return phi;
}

/* Motion cross product */
auto crm(const Vec6& a) {
    Mat6 out;
    out << vectorToSkewMat(a.head(3)), Eigen::Matrix3d::Zero(), 
           vectorToSkewMat(a.tail(3)), vectorToSkewMat(a.head(3));
    return out;    
}

/* Force cross product */
auto crf(const Vec6& a) {
    Mat6 out;
    out = -crm(a).transpose();
    return out;
}

};

#endif