#ifndef RIGIDBODYDYNAMICS_HPP
#define RIGIDBODYDYNAMICS_HPP

#include "RigidBodyModel.hpp"

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

class RigidBodyDynamics{
    private:
    RigidBodyModel _model;
    Vec6 _gravity;

    std::vector<JointType> _jointTypes;
    std::vector<CoordinateAxis> _jointAxes;
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<sMat> _Xup, _Xa, _X0;
    std::vector<sMat, Eigen::aligned_allocator<sMat>> _Xtree;
    vectorAligned<Vec6> _S, _v, _a, _ar, _a0, _c, _f, _fext, _p, _pc;
    std::vector<int> _parents;

    std::vector<SpatialInertia, Eigen::aligned_allocator<SpatialInertia>> _Ibody;
    vectorAligned<Mat6> _Ic;

    public: 

    Eigen::VectorXd genForce; 

    RigidBodyDynamics(const std::string &urdf_path);
    void init(Vec3 gravity);
    JointType getJointType(const std::string name);
    CoordinateAxis getJointAxis(const std::string name);
    void setState(const ModelState& dstate);
    void setDState(const ModelStateDerivative& dstate);
    void applyExternalForce(const int bodyId, const Vec3 &pos, const Vec6 &fext);
    // void applyExternalForce(const int jointId, const Vec3 &force);
    void floatingBaseInvDyn(const ModelState &state, const ModelStateDerivative &dstate);
    void fixedBaseInvDyn(const ModelState &state, const ModelStateDerivative &dstate);
    void inverseDynamics(const ModelState &state, const ModelStateDerivative &dstate);

};

#endif