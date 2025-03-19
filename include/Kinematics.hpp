#ifndef RIGIDBODYKINEMATICS_HPP
#define RIGIDBODYKINEMATICS_HPP

#include "modelParameters.hpp"

class RigidBodyKinematics {
    private:
    ModelParameters model;
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<tMat> _T, _T0;
    vectorAligned<sMat> _Xup, _Xa;
    vectorAligned<Vec6> _S;

    void setState(const ModelState& state);
    void setDState(const ModelStateDerivative& dstate);

    public:

    void initKinematics(ModelParameters urdf);
    Vec3 homogenousFK(const int& bodyId, const ModelState& state, const Vec3& pos);
    Vec3 forwardKinematics(const int& bodyId, const ModelState& state, const Vec3& pos);
    Eigen::MatrixXd bodyJacobian(const ModelState& state);

};

#endif