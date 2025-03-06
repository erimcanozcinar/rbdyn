#include "orientTools.hpp"

namespace ori {

    /* Converts CoordinateAxis into string */
    std::string toString(CoordinateAxis axis) {
        switch (axis) {
            case CoordinateAxis::X: return "X";
            case CoordinateAxis::Y: return "Y";
            case CoordinateAxis::Z: return "Z";
            case CoordinateAxis::None: return "None";
            default: return "Unknown";
        }
    }

    /* Constructs 3x3 rotation matrix from rotation axis and angle */
    RotMat coordinateRotation(CoordinateAxis axis, double theta) {
        double s = std::sin(theta);
        double c = std::cos(theta);
        RotMat R;
        if (axis == CoordinateAxis::X) {
            R << 1, 0, 0, 0, c, s, 0, -s, c;
        } else if (axis == CoordinateAxis::Y) {
            R << c, 0, -s, 0, 1, 0, s, 0, c;
        } else if (axis == CoordinateAxis::Z) {
            R << c, s, 0, -s, c, 0, 0, 0, 1;
        }
        return R;
    }

    /* Converts quaternion into 3x3 rotation matrix */
    RotMat quat2RotMat(const Quatd& quat) {
        Quatd q;
        q = quat.normalized();
        RotMat rMat;

        rMat << 2*(q.w()*q.w() + q.x()*q.x()) - 1, 2*(q.x()*q.y() + q.w()*q.z()), 2*(q.x()*q.z() - q.w()*q.y()),
                2*(q.x()*q.y() - q.w()*q.z()), 2*(q.w()*q.w() + q.y()*q.y()) - 1, 2*(q.y()*q.z() + q.w()*q.x()),
                2*(q.x()*q.z() + q.w()*q.y()), 2*(q.y()*q.z() - q.w()*q.x()), 2*(q.w()*q.w() + q.z()*q.z()) - 1;

        return rMat;
    }

    /* Converts 3x3 vector into skew symmetric matrix */
    Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        return m;
    }

    Eigen::Vector3d skewMatToVector(const Eigen::Matrix3d& m) {
        Eigen::Vector3d v;
        v << m(2,1), m(0,2), m(1,0);
        return v;
    }

    /* Converts skew symmetric matrix into 3x3 vector */
    Eigen::Vector3d skewMatToVec(const Eigen::Matrix3d& m) {
    return 0.5 * Eigen::Vector3d(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                                            (m(1, 0) - m(0, 1)));
    }

}