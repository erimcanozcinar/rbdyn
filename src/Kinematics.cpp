#include "Kinematics.hpp"

void RigidBodyKinematics::initKinematics(ModelParameters urdf) {
    model = urdf;
    Mat6 eye6 = Mat6::Identity();
    Vec6 zero6 = Vec6::Zero();
    Mat6 zero66 = Mat6::Zero();
    Mat4 zero44 = Mat4::Zero();

    for(int i=0; i<model.nBody+1; i++) {
        _Xup.push_back(zero66);
        _Xa.push_back(zero66);
        _T.push_back(zero44);
        _T0.push_back(zero44);
        _S.push_back(zero6);
    }
}

void RigidBodyKinematics::setState(const ModelState& state) {
    _state = state;
}

void RigidBodyKinematics::setDState(const ModelStateDerivative& dstate) {
    _dstate = dstate;
}

Vec3 RigidBodyKinematics::homogenousFK(const int& bodyId, const ModelState& state, const Vec3& pos) {
    setState(state);
    
    sMat Xj = sMat::Zero(); 
    Xj = jointSpatialTransform(model._jointTypes[0],model._jointAxes[0], 0);
    _Xup[0] = Xj*model._Xtree[0];
    _T[0] = plücker2Homogeneous(_Xup[0]);
    _T0[0] = _T[0];

    for(int i = 1; i <= bodyId; i++) {
        Xj = jointSpatialTransform(model._jointTypes[i],model._jointAxes[i], _state.q[i-1]);
        _Xup[i] = Xj*model._Xtree[i];
        _T[i] = plücker2Homogeneous(_Xup[i]);
        _T0[i] = _T0[model._parents[i]]*_T[i];
    }

    _T[bodyId+1] = homogeneousTransform(RotMat::Identity(), pos);
    _T0[bodyId+1] = _T0[bodyId]*_T[bodyId+1];

    std::cout << _T0[bodyId+1].topRightCorner<3,1>() << std::endl;
    return _T0[bodyId+1].topRightCorner<3,1>();
}

Vec3 RigidBodyKinematics::forwardKinematics(const int& bodyId, const ModelState& state, const Vec3& pos) {
    setState(state);

    Vec3 P = Vec3::Zero();
    
    sMat Xj = sMat::Zero(); 
    Xj = jointSpatialTransform(model._jointTypes[0],model._jointAxes[0], 0);
    _Xa[0] = Xj*model._Xtree[0];

    for(int i = 1; i <= bodyId; i++) {
        Xj = jointSpatialTransform(model._jointTypes[i],model._jointAxes[i], _state.q[i-1]);
        _Xa[i] = Xj*model._Xtree[i];
        _Xa[i] = _Xa[i]*_Xa[model._parents[i]];
    }

    sMat Xai = _Xa[bodyId].inverse();
    RotMat Ri = rotationFromX(Xai);
    Vec3 ri = translationFromX(Xai);
    P = Ri*(pos - ri);
    return P;
}

Eigen::MatrixXd RigidBodyKinematics::bodyJacobian(const ModelState& state) {
    setState(state);

    JacMat J;
    J.resize(6,model.nBody);
    J.setZero();
    sMat Xj = sMat::Zero(); 
    Xj = jointSpatialTransform(model._jointTypes[0],model._jointAxes[0], 0);
    _S[0] = jointMotionSubspace(model._jointTypes[0], model._jointAxes[0]);
    _Xa[0] = Xj*model._Xtree[0];
    J.template block<6,1>(0,0) = _Xa[0].inverse()*_S[0];

    for(int i = 1; i < model.nBody; i++) {
        Xj = jointSpatialTransform(model._jointTypes[i],model._jointAxes[i], _state.q[i-1]);
        _S[i] = jointMotionSubspace(model._jointTypes[i], model._jointAxes[i]);
        _Xa[i] = Xj*model._Xtree[i];
        if(model._parents[i] != 0 && model._parents[i] != -1)
            _Xa[i] = _Xa[i]*_Xa[model._parents[i]];
        
        J.block<6,1>(0,i) = _Xa[i].inverse()*_S[i];
    }
    J = _Xa[2]*_Xa[1]*_Xa[0]*J;
    // std::cout << J << std::endl;
    // std::cout << "---------------" << std::endl;
    return J.block<3,2>(3,1);
}

