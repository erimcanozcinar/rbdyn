#ifndef RIGIDBODYDYNAMICS_HPP
#define RIGIDBODYDYNAMICS_HPP

#include "modelParameters.hpp"

class RigidBodyDynamics {
    private:
    ModelParameters model;
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<sMat> _Xup, _Xa, _X0;
    vectorAligned<Vec6> _S, _v, _a, _ar, _a0, _c, _f, _fext, _p, _pc;
    vectorAligned<Mat6> _Ic;

    Eigen::VectorXd genF;
    Eigen::VectorXd genForce;

    void setState(const ModelState& state);
    void setDState(const ModelStateDerivative& dstate);
    void setJointAngles(int i, int &jIndex);
    void forceSelectionMatrix();

    protected:
    void initDynamics(ModelParameters urdf);        
    void floatingBaseInvDyn();
    void fixedBaseInvDyn();
    
    public: 


    RigidBodyDynamics() = default;
    void applyExternalForce(const int bodyId, const Vec6 &fext, const Vec3 &pos = Vec3::Zero());
    Eigen::VectorXd inverseDynamics(const ModelState &state, const ModelStateDerivative &dstate);

};

#endif