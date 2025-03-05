#ifndef MODELPARAMETERS_HPP
#define MODELPARAMETERS_HPP

#include "SpatialInertia.hpp"
using namespace spatial;

struct ModelParameters {
    protected:
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

    
};

#endif