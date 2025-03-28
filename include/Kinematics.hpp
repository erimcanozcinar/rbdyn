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
    vectorAligned<Vec6> _S, _J;

    ModelState stateIK;


    void setState(const ModelState& state);
    void setDState(const ModelStateDerivative& dstate);
    void setJointAngles(int i, int &jIndex);

    public:

    void initKinematics(ModelParameters urdf);
    Vec3 homogenousFK(const int& bodyId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Vec3 forwardKinematic(const int& bodyId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Eigen::MatrixXd bodyJacobian(const int& bodyId, const ModelState& state, const Vec3& pos = Vec3::Zero());
    Eigen::VectorXd inverseKinematic(const std::vector<int>& bodyId, const std::vector<Vec3>& desPos, const double& err_tol=1e-5, const int& max_iter=10);

};

#endif