#include "Dynamics.hpp"

RigidBodyDynamics::RigidBodyDynamics() {
    
}

void RigidBodyDynamics::initDynamics(ModelParameters urdf) {
    model = urdf;
    Mat6 eye6 = Mat6::Identity();
    Vec6 zero6 = Vec6::Zero();
    Mat6 zero66 = Mat6::Zero();

    SpatialInertia zeroInertia(zero66);
    for(int i = 0; i < model.nBody; i++) {
        _v.push_back(zero6);
        _a.push_back(zero6);
        _a0.push_back(zero6);
        _ar.push_back(zero6);
        _c.push_back(zero6);
        _S.push_back(zero6);
        _f.push_back(zero6);
        _fext.push_back(zero6);
        _pc.push_back(zero6);
        _Xup.push_back(eye6);
        _Xa.push_back(eye6);
        _X0.push_back(eye6);
        _Ic.push_back(zero66);
    }
    
    if(model._jointTypes[0] == JointType::Fixed) {
        genF.resize(model.nBody);
        genForce.resize(model.nDof);
    } else {
        genF.resize(model.nBody+5);
        genForce.resize(model.nDof);
    }
    genF.setZero(); 
    genForce.setZero(); 
}

void RigidBodyDynamics::setState(const ModelState& state) {
    _state = state;
}

void RigidBodyDynamics::setDState(const ModelStateDerivative& dstate) {
    _dstate = dstate;
}

void RigidBodyDynamics::setJointAngles(int i, int &jIndex) {
    if(model._jointTypes[i] != JointType::Fixed) {
        model._q[i] = _state.q[jIndex];
        model._dq[i] = _state.dq[jIndex];
        model._ddq[i] = _dstate.ddq[jIndex];
        jIndex++;
    }    
}

/** Apply external torque and force on body in world frame.

* @param[in] bodyId Body index. It can be retrieved by getLinkID() or getBodyIdx().
* @param[in] fext Torques and forces acting on body in world frame. (fext = [torques; forces])
* @param[in] pos Position of point with respec to body frame where the force is applied */
void RigidBodyDynamics::applyExternalForce(const int bodyId, const Vec3 &pos, const Vec6 &fext) {
    sMat X = sMat::Zero();
    Vec3 pos_w = _X0[bodyId].topLeftCorner<3,3>().transpose()*pos;
    X.topLeftCorner<3,3>() = _X0[bodyId].topLeftCorner<3,3>();
    X.bottomLeftCorner<3,3>() = _X0[bodyId].topLeftCorner<3,3>()*vectorToSkewMat(pos_w);
    X.bottomRightCorner<3,3>() = _X0[bodyId].bottomRightCorner<3,3>();
    _fext[bodyId] = (X.transpose()).inverse()*fext;
}

// /** Apply external force on joint in world frame.
//  * Note: this method does not apply torque on joint. This method only apply force. 
//  * If you want to apply torque on joint, simply apply torque on child body of the joint 
// * @param[in] jointId Body index. It can be retrieved by getLinkID() or getBodyIdx().
// * @param[in] force Forces acting on joint in body frame. (force = [forces]) */
// void RigidBodyDynamics::applyExternalForce(const int jointId, const Vec3 &force) {
//     sMat X = sMat::Zero();
//     Vec6 fext = Vec6::Zero();
//     fext.bottomRows<3>() = force;
//     int bodyId = _model.jointParentIDs[jointId];
//     X.topLeftCorner<3,3>() = _X0[bodyId].topLeftCorner<3,3>();
//     X.bottomLeftCorner<3,3>() = _X0[bodyId].topLeftCorner<3,3>()*vectorToSkewMat(_model.jointLocations[jointId]);
//     X.bottomRightCorner<3,3>() = _X0[bodyId].bottomRightCorner<3,3>();
//     _fext[bodyId] = (X.transpose()).inverse()*fext;
// }

void RigidBodyDynamics::floatingBaseInvDyn() {

    sMat Xfb = sMat::Zero();
    // Xfb << _state.baseR, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), _state.baseR;
    _Xup[0] = spatialTransform(_state.baseR, _state.basePosition)*model._Xtree[0];
    _X0[0] = _Xup[0];
    _v[0] = _Xup[0]*_state.baseVelocity;
    _ar[0] = -model._gravity;

    int jIndex = 0;
    for(int i = 1; i < model.nBody; i++) {
        setJointAngles(i, jIndex);
        Vec6 vJ = Vec6::Zero();
        sMat Xj = sMat::Zero();
        Xj = jointSpatialTransform(model._jointTypes[i], model._jointAxes[i], model._jointAxisCoef[i]*model._q[i]);
        _Xup[i] = Xj*model._Xtree[i];
        _X0[i] = _Xup[i]*_X0[model._parents[i]];
        _S[i] = jointMotionSubspace(model._jointTypes[i], model._jointAxes[i], model._jointAxisCoef[i]);
        vJ = _S[i]*model._dq[i];
        _v[i] = _Xup[i]*_v[model._parents[i]] + vJ;

        _ar[i] = _Xup[i] * _ar[model._parents[i]] + crm(_v[i])*vJ + _S[i] * model._ddq[i];
        _Ic[i] = model._Ibody[i].getMatrix();
        _pc[i] = model._Ibody[i].getMatrix() * _ar[i] + crf(_v[i])*model._Ibody[i].getMatrix()*_v[i] - _fext[i];
    }
    
    _Ic[0] = model._Ibody[0].getMatrix();
    _pc[0] = model._Ibody[0].getMatrix()*_ar[0] + crf(_v[0])*model._Ibody[0].getMatrix()*_v[0] - _fext[0];
    for(int i = model.nBody-1; i>0; i--) {
        _Ic[model._parents[i]] = _Ic[model._parents[i]] + _Xup[i].transpose()*_Ic[i]*_Xup[i];
        _pc[model._parents[i]] = _pc[model._parents[i]] + _Xup[i].transpose()*_pc[i];
    }

    _a0[0] = -_Ic[0].inverse()*_pc[0];
    genF.template head<6>() = _Ic[0]*_a0[0] + _pc[0];
    for(int i = 1; i < model.nBody; i++) {
        _a0[i] = _Xup[i]*_a0[model._parents[i]];
        genF(i+5) = _S[i].transpose()*(_Ic[i]*_a0[i] + _pc[i]); 
    }       
    
    genForce = model._Sf*genF;
}

void RigidBodyDynamics::fixedBaseInvDyn() {
    sMat Xj = sMat::Zero();
    Xj = jointSpatialTransform(model._jointTypes[0], model._jointAxes[0], 0);
    _Xup[0] = Xj*model._Xtree[0];
    _X0[0] = _Xup[0];
    _S[0] = jointMotionSubspace(model._jointTypes[0], model._jointAxes[0], model._jointAxisCoef[0]);
    Vec6 vJ = _S[0]*0;
    _v[0] = Vec6::Zero();
    _c[0] = Vec6::Zero();
    _a[0] = -model._gravity;
    _f[0] = model._Ibody[0].getMatrix() * _a[0] + crf(_v[0])*model._Ibody[0].getMatrix()*_v[0] - _fext[0];

    int jIndex = 0;
    for(int i = 1; i < model.nBody; i++) {
        setJointAngles(i,jIndex);
        Xj = sMat::Zero();
        Xj = jointSpatialTransform(model._jointTypes[i], model._jointAxes[i], model._q[i]);
        _Xup[i] = Xj*model._Xtree[i];
        _X0[i] = _Xup[i]*_X0[model._parents[i]];
        _S[i] = jointMotionSubspace(model._jointTypes[i], model._jointAxes[i], model._jointAxisCoef[i]);
        vJ = _S[i]*model._dq[i];
        _v[i] = _Xup[i]*_v[model._parents[i]] + vJ;
        _a[i] = _Xup[i] * _a[model._parents[i]] + crm(_v[i])*vJ + _S[i] * model._ddq[i];
        _f[i] = model._Ibody[i].getMatrix() * _a[i] + crf(_v[i])*model._Ibody[i].getMatrix()*_v[i] - _fext[i];
    }

    for(int i=model.nBody-1; i>0; i--) {
        genF[i] = _S[i].dot(_f[i]);
        if(model._parents[i] != 0) {
            _f[model._parents[i]] = _f[model._parents[i]] + _Xup[i].transpose()*_f[i];
        }
    }
    genForce = model._Sf*genF;
}

/** Solves inverse dynamics of system. Output of this function is genForce.
 *  @param[in] state States of the system.
 *  @param[in] dstate Derivative of states of the system.
 */
void RigidBodyDynamics::inverseDynamics(const ModelState &state, const ModelStateDerivative &dstate) {
    setState(state);
    setDState(dstate); 
    if(model._jointTypes[0] == JointType::Floating) {
        floatingBaseInvDyn();
    } else if(model._jointTypes[0] == JointType::Fixed) {
        fixedBaseInvDyn();
    } else {
        std::cout << "Unknown base, inverse Dynamics can not be solved" << std::endl;
    }
}