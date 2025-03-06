#ifndef RIGIDBODYKINEMATICS_HPP
#define RIGIDBODYKINEMATICS_HPP

#include "modelParameters.hpp"

class RigidBodyKinematics : public virtual ModelParameters{
    private:
    ModelState _state;
    ModelStateDerivative _dstate;
    vectorAligned<tMat> _T, _T0;
    vectorAligned<sMat> _Xup;

    void setState(const ModelState& state);
    void setDState(const ModelStateDerivative& dstate);

    public:

    void initKinematics();
    void forwardKinematics(const int& bodyID, const ModelState& state);

};

#endif