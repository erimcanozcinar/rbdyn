#ifndef RIGIDBODYKINEMATICS_HPP
#define RIGIDBODYKINEMATICS_HPP

#include "modelParameters.hpp"

class RigidBodyKinematics {
    private:
    ModelParameters model;
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<tMat> _T, _T0;
    vectorAligned<sMat> _Xup, _Xa, _Xo;
    vectorAligned<Vec6> _S;

    ModelState stateIK;
    bool ik_init = false;


    void setState(const ModelState& state);
    void setDState(const ModelStateDerivative& dstate);
    void setJointAngles(int i, int &jIndex);

    public:

    RigidBodyKinematics() = default;
    void initKinematics(ModelParameters urdf);
    Vec3 homogenousFK(const int& frameId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Vec3 forwardKinematic(const int& frameId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Eigen::MatrixXd pointJacobian(const int& frameId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Eigen::VectorXd inverseKinematic(const std::vector<int>& frameId, const std::vector<Vec3>& desPos, const Eigen::VectorXd &Q_init, const Vec3 &basePos,
                                                      const RotMat &baseRot, const double& err_tol=1e-5, const int& max_iter=10);

};

#endif