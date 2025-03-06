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
        SpatialInertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia);

         /* Construct spatial inertia from 6x6 matrix */
        inline explicit SpatialInertia(const Mat6& inertia) { _inertia = inertia; }

        /* If no argument is given, zero. */
        inline SpatialInertia() { _inertia = Mat6::Zero(); }

        /* Construct spatial inertia from mass property vector */
        explicit SpatialInertia(const Eigen::Matrix<double, 10, 1>& a);

        /*!
        * Construct spatial inertia from pseudo-inertia. This is described in
        * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
        *   Identification: A Statistical Perspective on the Mass Distribution, by
        *   Wensing, Kim, Slotine
        * @param P
        */
        explicit SpatialInertia(const Eigen::Matrix4d& P);

        /*!
        * Convert spatial inertia to mass property vector
        */
        Eigen::Matrix<double, 10, 1> asMassPropertyVector();

        /*!
        * Get 6x6 spatial inertia
        */
        inline const Mat6& getMatrix() const { return _inertia; }

        inline void setMatrix(const Mat6& mat) { _inertia = mat; }

        inline void addMatrix(const Mat6& mat) { _inertia += mat; }

        /*!
        * Get mass
        */
        inline double getMass() { return _inertia(5, 5); }

        /*!
        * Get center of mass location
        */
        Eigen::Vector3d getCOM();

        /*!
        * Get 3x3 rotational inertia
        */
        Eigen::Matrix3d getInertiaTensor();

        /*!
        * Convert to 4x4 pseudo-inertia matrix.  This is described in
        * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
        *   Identification: A Statistical Perspective on the Mass Distribution, by
        *   Wensing, Kim, Slotine
        */
        Eigen::Matrix4d getPseudoInertia();

        /*!
        * Flip inertia matrix around an axis.  This isn't efficient, but it works!
        */
        SpatialInertia flipAlongAxis(CoordinateAxis axis);


};

#endif