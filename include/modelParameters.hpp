#ifndef MODELPARAMETERS_HPP
#define MODELPARAMETERS_HPP

#include "SpatialInertia.hpp"
using namespace spatial;

struct ModelState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d basePosition;
    Eigen::Quaterniond baseOrientation;
    RotMat baseR;
    Vec6 baseVelocity;  // body coordinates
    Eigen::Matrix<double, -1, 1> q;
    Eigen::Matrix<double, -1, 1>  dq;
};

struct ModelStateDerivative {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d dBasePosition;
    Vec6 dBaseVelocity;
    Eigen::Matrix<double, -1, 1> ddq;
};

struct ModelParameters {
    int nBody;
    int nDof;

    Vec6 _gravity;

    std::vector<std::string> _linkNames;
    std::vector<int> _linkIDs;
    std::vector<Eigen::Vector3d> _linkCOMs;
    std::vector<Eigen::Vector3d> _linkRots;
    std::vector<double> _linkMasses;
    std::vector<Eigen::Matrix3d> _linkInertias;

    std::vector<std::string> _jointNames;
    std::vector<int> _jointIDs;
    std::vector<std::string> _jointParents;
    std::vector<int> _jointParentIDs;
    std::vector<std::string> _jointChilds;    
    std::vector<int> _jointChildIDs;
    std::vector<Eigen::Vector3d> _jointLocations;
    std::vector<Eigen::Vector3d> _jointRotations;

    std::vector<JointType> _jointTypes;
    std::vector<CoordinateAxis> _jointAxes;    
    std::vector<sMat, Eigen::aligned_allocator<sMat>> _Xtree;
    std::vector<int> _parents;
    std::vector<SpatialInertia, Eigen::aligned_allocator<SpatialInertia>> _Ibody;
    Eigen::MatrixXd _Sf;
    std::vector<double> _q, _dq, _ddq;

    
};

#endif