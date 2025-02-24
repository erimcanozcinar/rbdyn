#ifndef SPATIALINERTIA_HPP
#define SPATIALINERTIA_HPP


#include "Spatial.hpp"


using namespace spatial;

class SpatialInertia {
    private:
        Mat6 _inertia;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /* Construct spatial inertia from mass, center of mass, and 3x3 rotational inertia */
        SpatialInertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia) {
            Eigen::Matrix3d cSkew = vectorToSkewMat(com);
            _inertia.template topLeftCorner<3, 3>() =
                inertia + mass * cSkew * cSkew.transpose();
            _inertia.template topRightCorner<3, 3>() = mass * cSkew;
            _inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
            _inertia.template bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
        }

         /* Construct spatial inertia from 6x6 matrix */
        explicit SpatialInertia(const Mat6& inertia) { _inertia = inertia; }

        /* If no argument is given, zero. */
        SpatialInertia() { _inertia = Mat6::Zero(); }

        /* Construct spatial inertia from mass property vector */
        explicit SpatialInertia(const Eigen::Matrix<double, 10, 1>& a) {
            _inertia(0, 0) = a(4);
            _inertia(0, 1) = a(9);
            _inertia(0, 2) = a(8);
            _inertia(1, 0) = a(9);
            _inertia(1, 1) = a(5);
            _inertia(1, 2) = a(7);
            _inertia(2, 0) = a(8);
            _inertia(2, 1) = a(7);
            _inertia(2, 2) = a(6);
            Eigen::Matrix3d cSkew = vectorToSkewMat(Eigen::Vector3d(a(1), a(2), a(3)));
            _inertia.template topRightCorner<3, 3>() = cSkew;
            _inertia.template bottomLeftCorner<3, 3>() = cSkew.transpose();
            _inertia.template bottomRightCorner<3, 3>() = a(0) * Eigen::Matrix3d::Identity();
        }

        /*!
        * Construct spatial inertia from pseudo-inertia. This is described in
        * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
        *   Identification: A Statistical Perspective on the Mass Distribution, by
        *   Wensing, Kim, Slotine
        * @param P
        */
        explicit SpatialInertia(const Eigen::Matrix4d& P) {
            Mat6 I;
            double m = P(3, 3);
            Eigen::Vector3d h = P.template topRightCorner<3, 1>();
            Eigen::Matrix3d E = P.template topLeftCorner<3, 3>();
            Eigen::Matrix3d Ibar = E.trace() * Eigen::Matrix3d::Identity() - E;
            I.template topLeftCorner<3, 3>() = Ibar;
            I.template topRightCorner<3, 3>() = vectorToSkewMat(h);
            I.template bottomLeftCorner<3, 3>() = vectorToSkewMat(h).transpose();
            I.template bottomRightCorner<3, 3>() = m * Eigen::Matrix3d::Identity();
            _inertia = I;
        }

        /*!
        * Convert spatial inertia to mass property vector
        */
        Eigen::Matrix<double, 10, 1> asMassPropertyVector() {
            Eigen::Matrix<double, 10, 1> a;
            Eigen::Vector3d h = skewMatToVec(_inertia.template topRightCorner<3, 3>());
            a << _inertia(5, 5), h(0), h(1), h(2), _inertia(0, 0), _inertia(1, 1),
                _inertia(2, 2), _inertia(2, 1), _inertia(2, 0), _inertia(1, 0);
            return a;
        }

        /*!
        * Get 6x6 spatial inertia
        */
        const Mat6& getMatrix() const { return _inertia; }

        void setMatrix(const Mat6& mat) { _inertia = mat; }

        void addMatrix(const Mat6& mat) { _inertia += mat; }

        /*!
        * Get mass
        */
        double getMass() { return _inertia(5, 5); }

        /*!
        * Get center of mass location
        */
        Eigen::Vector3d getCOM() {
            double m = getMass();
            Eigen::Matrix3d mcSkew = _inertia.template topRightCorner<3, 3>();
            Eigen::Vector3d com = skewMatToVec(mcSkew) / m;
            return com;
        }

        /*!
        * Get 3x3 rotational inertia
        */
        Eigen::Matrix3d getInertiaTensor() {
            double m = getMass();
            Eigen::Matrix3d mcSkew = _inertia.template topRightCorner<3, 3>();
            Eigen::Matrix3d I_rot = _inertia.template topLeftCorner<3, 3>() -
                            mcSkew * mcSkew.transpose() / m;
            return I_rot;
        }

        /*!
        * Convert to 4x4 pseudo-inertia matrix.  This is described in
        * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
        *   Identification: A Statistical Perspective on the Mass Distribution, by
        *   Wensing, Kim, Slotine
        */
        Eigen::Matrix4d getPseudoInertia() {
            Eigen::Vector3d h = skewMatToVec(_inertia.template topRightCorner<3, 3>());
            Eigen::Matrix3d Ibar = _inertia.template topLeftCorner<3, 3>();
            double m = _inertia(5, 5);
            Eigen::Matrix4d P;
            P.template topLeftCorner<3, 3>() =
                0.5 * Ibar.trace() * Eigen::Matrix3d::Identity() - Ibar;
            P.template topRightCorner<3, 1>() = h;
            P.template bottomLeftCorner<1, 3>() = h.transpose();
            P(3, 3) = m;
            return P;
        }

        /*!
        * Flip inertia matrix around an axis.  This isn't efficient, but it works!
        */
        SpatialInertia flipAlongAxis(CoordinateAxis axis) {
            Eigen::Matrix4d P = getPseudoInertia();
            Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
            if (axis == CoordinateAxis::X)
            X(0, 0) = -1;
            else if (axis == CoordinateAxis::Y)
            X(1, 1) = -1;
            else if (axis == CoordinateAxis::Z)
            X(2, 2) = -1;
            P = X * P * X;
            return SpatialInertia(P);
        }


};

#endif