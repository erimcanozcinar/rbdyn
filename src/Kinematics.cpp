#include "Kinematics.hpp"

void RigidBodyKinematics::initKinematics() {
    Mat6 eye6 = Mat6::Identity();
    Vec6 zero6 = Vec6::Zero();
    Mat6 zero66 = Mat6::Zero();
    Mat4 zero44 = Mat4::Zero();

    for(int i=0; i<nBody; i++) {
        _Xup.push_back(zero66);
        _T.push_back(zero44);
        _T0.push_back(zero44);
    }
}

void RigidBodyKinematics::setState(const ModelState& state) {
    _state = state;
}

void RigidBodyKinematics::setDState(const ModelStateDerivative& dstate) {
    _dstate = dstate;
}

void RigidBodyKinematics::forwardKinematics(const int& bodyID, const ModelState& state) {
    setState(state);
    
    sMat Xj = sMat::Zero(); 
    Xj = jointSpatialTransform(_jointTypes[0],_jointAxes[0], 0);
    _Xup[0] = Xj*_Xtree[0];
    _T[0] = plücker2Homogeneous(_Xup[0]);
    _T0[0] = _T[0];

    for(int i = 1; i < nBody; i++) {
        Xj = jointSpatialTransform(_jointTypes[i],_jointAxes[i], _state.q[i-1]);
        _Xup[i] = Xj*_Xtree[i];
        _T[i] = plücker2Homogeneous(_Xup[i]);
        _T0[i] = _T[i]*_T[_parents[i]];

        std::cout << _T[i] << std::endl;
        std::cout << "--------" << std::endl;
    }
    std::cout << "******" << std::endl;
}