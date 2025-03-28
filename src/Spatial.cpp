#include "Spatial.hpp"

namespace spatial {
using namespace ori;

/* Converts JointType into string */
std::string toString(JointType joint) {
    switch (joint) {
        case JointType::Prismatic: return "Prismatic";
        case JointType::Revolute: return "Revolute";
        case JointType::Spherical: return "Spherical";
        case JointType::Floating: return "Floating";
        case JointType::Continuous: return "Continuous";
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
sMat spatialTransform(const RotMat& E, const Vec3& r) {
    sMat X = sMat::Zero();
    X.template topLeftCorner<3, 3>() = E;
    X.template bottomRightCorner<3, 3>() = E;
    X.template bottomLeftCorner<3, 3>() = -E * vectorToSkewMat(r);
    return X;
}

RotMat rotationFromX(const sMat& X) {
    RotMat E = X.template topLeftCorner<3,3>();
    return E;
}

Vec3 translationFromX(const sMat& X) {
    RotMat E = rotationFromX(X);
    Vec3 r = -skewMatToVector(E.transpose()*X.template bottomLeftCorner<3, 3>());
    return r;
}

sMat invertX(const sMat& X) {
    RotMat E = rotationFromX(X);
    Vec3 r = -skewMatToVector(E.transpose()*X.template bottomLeftCorner<3, 3>());
    sMat X_inv = sMat::Zero();
    X_inv.template topLeftCorner<3, 3>() = E.transpose();
    X_inv.template bottomRightCorner<3, 3>() = E.transpose();
    X_inv.template bottomLeftCorner<3, 3>() = vectorToSkewMat(r)*E.transpose();
    return X_inv;
}

tMat plücker2Homogeneous(const sMat& X) {
    RotMat E = X.topLeftCorner<3, 3>();
    RotMat R = E.transpose();
    Mat3 Erx = -X.bottomLeftCorner<3,3>();
    Mat3 rx = E.transpose()*Erx;
    Vec3 r;
    r << rx(2,1), rx(0,2), rx(1,0);
    tMat T = tMat::Identity();
    T.template topLeftCorner<3,3>() = R;
    T.template topRightCorner<3,1>() = r;
    return T;
}

tMat homogeneousTransform(const RotMat& R, const Vec3& r) {
    tMat T = tMat::Identity();
    T.template topLeftCorner<3,3>() = R;
    T.template topRightCorner<3,1>() = r;
    return T;
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
Vec6 jointMotionSubspace(JointType joint, CoordinateAxis axis, int c) {
    Eigen::Vector3d v(0, 0, 0);
    Vec6 phi = Vec6::Zero();
    if (axis == CoordinateAxis::X)
        v(0) = c*1;
    else if (axis == CoordinateAxis::Y)
        v(1) = c*1;
    else
        v(2) = c*1;

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
Mat6 crm(const Vec6& a) {
    Mat6 out;
    out << vectorToSkewMat(a.head(3)), Eigen::Matrix3d::Zero(), 
           vectorToSkewMat(a.tail(3)), vectorToSkewMat(a.head(3));
    return out;    
}

/* Force cross product */
Mat6 crf(const Vec6& a) {
    Mat6 out;
    out = -crm(a).transpose();
    return out;
}

};