#ifndef RIGIDBODYDYNAMICS_HPP
#define RIGIDBODYDYNAMICS_HPP

#include "modelParameters.hpp"

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

class RigidBodyDynamics : public virtual ModelParameters {
    private:
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<sMat> _Xup, _Xa, _X0;
    vectorAligned<Vec6> _S, _v, _a, _ar, _a0, _c, _f, _fext, _p, _pc;
    vectorAligned<Mat6> _Ic;
    public: 

    Eigen::VectorXd genForce; 

    RigidBodyDynamics();
    void initDynamics();
    void setState(const ModelState& dstate);
    void setDState(const ModelStateDerivative& dstate);
    void applyExternalForce(const int bodyId, const Vec3 &pos, const Vec6 &fext);
    // void applyExternalForce(const int jointId, const Vec3 &force);
    void floatingBaseInvDyn(const ModelState &state, const ModelStateDerivative &dstate);
    void fixedBaseInvDyn(const ModelState &state, const ModelStateDerivative &dstate);
    void inverseDynamics(const ModelState &state, const ModelStateDerivative &dstate);

};

#endif