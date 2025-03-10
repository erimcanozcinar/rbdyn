#include "Kinematics.hpp"

void RigidBodyKinematics::initKinematics() {
    Mat6 eye6 = Mat6::Identity();
    Vec6 zero6 = Vec6::Zero();
    Mat6 zero66 = Mat6::Zero();
    Mat4 zero44 = Mat4::Zero();

    for(int i=0; i<nBody+1; i++) {
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

Vec3 RigidBodyKinematics::forwardKinematics(const int& bodyId, const ModelState& state, const Vec3& pos) {
    setState(state);
    
    sMat Xj = sMat::Zero(); 
    Xj = jointSpatialTransform(_jointTypes[0],_jointAxes[0], 0);
    _Xup[0] = Xj*_Xtree[0];
    _T[0] = plücker2Homogeneous(_Xup[0]);
    _T0[0] = _T[0];

    for(int i = 1; i <= bodyId; i++) {
        Xj = jointSpatialTransform(_jointTypes[i],_jointAxes[i], _state.q[i-1]);
        _Xup[i] = Xj*_Xtree[i];
        _T[i] = plücker2Homogeneous(_Xup[i]);
        _T0[i] = _T0[_parents[i]]*_T[i];
    }

    _T[bodyId+1] = homogeneousTransform(RotMat::Identity(), pos);
    _T0[bodyId+1] = _T0[bodyId]*_T[bodyId+1];

    std::cout << _T0[bodyId+1].topRightCorner<3,1>() << std::endl;
    return _T0[bodyId+1].topRightCorner<3,1>();
}

