#ifndef SPATIAL_HPP
#define SPATIAL_HPP

#include "orientTools.hpp"

namespace spatial {
using namespace ori;
enum class JointType { Prismatic, Revolute, Spherical, Floating, Fixed };

/* Converts JointType into string */
std::string toString(JointType joint);

/* Constructs Plücker coordinate transformation matrix for spesific axis of rotation
(translation part is zero) */
sMat spatialRotation(CoordinateAxis axis, double theta);

sMat spatialTranslation(const Vec3 &r);

/* Constructs a Plücker coordinate transformation matrix (6x6) */
sMat spatialTransform(const RotMat& E, const Vec3& r);

tMat plücker2Homogeneous(const sMat& X);

tMat homogeneousTransform(const RotMat& R, const Vec3& r);

/* Constructs a Plücker coordinate transformation matrix for specific joint type */
sMat jointSpatialTransform(JointType joint, CoordinateAxis axis, double q);

/* Constructs joint motion subspace vector */
Vec6 jointMotionSubspace(JointType joint, CoordinateAxis axis);

/* Motion cross product */
Mat6 crm(const Vec6& a);

/* Force cross product */
Mat6 crf(const Vec6& a);

};

#endif