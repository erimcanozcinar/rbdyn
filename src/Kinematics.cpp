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
        _Xo.push_back(zero66);
        _T.push_back(zero44);
        _T0.push_back(zero44);
        _S.push_back(zero6);
    }

    if(model._jointTypes[0] == JointType::Floating)
        stateIK.q.resize(model.nDof-6);
    else
        stateIK.q.resize(model.nDof);

    stateIK.q.setZero();
}

void RigidBodyKinematics::setState(const ModelState& state) {
    _state = state;
}

void RigidBodyKinematics::setDState(const ModelStateDerivative& dstate) {
    _dstate = dstate;
}

void RigidBodyKinematics::setJointAngles(int i, int &jIndex) {
    if(model._jointTypes[i] != JointType::Fixed) {
        model._q[i] = _state.q[jIndex];
        // model._dq[i] = _state.dq[jIndex];
        // model._ddq[i] = _dstate.ddq[jIndex];
        jIndex++;
    }    
}

/** Solves forward kinematics for a given position by using homogenous transformation.
 * @param[in] bodyId Body index. It can be retrieved by getLinkID() or getBodyIdx().
 * @param[in] state Robots states.
 * @param[in] pos Desired point position wrt body frame. (By default it is set to zero)
 * @param[out] out Position vector of point wrt world frame.
 */
Vec3 RigidBodyKinematics::homogenousFK(const int& bodyId, const ModelState& state, const Vec3& pos) {
    setState(state);
    int jIndex = 0;
    for(int i = 1; i < model.nBody; i++)
        setJointAngles(i,jIndex);
    
    sMat Xj = sMat::Zero();
    if(model._jointTypes[0] == JointType::Fixed) { 
        Xj = jointSpatialTransform(model._jointTypes[0],model._jointAxes[0], 0);
        _Xup[0] = Xj*model._Xtree[0];
        _T[0] = plücker2Homogeneous(_Xup[0]);
        _T0[0] = _T[0];
    } else if(model._jointTypes[0] == JointType::Floating) {
        _Xup[0] = spatialTransform(_state.baseR, _state.basePosition);
        _T[0] = plücker2Homogeneous(_Xup[0]);
        _T0[0] = _T[0];
    }

    for(int i : model._pathJoints[bodyId]) {
        Xj = jointSpatialTransform(model._jointTypes[i],model._jointAxes[i], model._jointAxisCoef[i]*model._q[i]);
        _Xup[i] = Xj*model._Xtree[i];
        _T[i] = plücker2Homogeneous(_Xup[i]);
        _T0[i] = _T0[model._parents[i]]*_T[i];
    }

    _T[bodyId+1] = homogeneousTransform(RotMat::Identity(), pos);
    _T0[bodyId+1] = _T0[bodyId]*_T[bodyId+1];

    return _T0[bodyId+1].topRightCorner<3,1>();
}

/** Solves forward kinematics for a given point by using spatial transformation.
 * @param[in] bodyId Body index. It can be retrieved by getLinkID() or getBodyIdx().
 * @param[in] state Robots states.
 * @param[in] pos Desired point position wrt body frame. (By default it is set to zero)
 * @param[out] out Position vector of point wrt world frame.
 */
Vec3 RigidBodyKinematics::forwardKinematic(const int& bodyId, const ModelState& state, const Vec3& pos) {
    setState(state);
    int jIndex = 0;
    for(int i = 1; i < model.nBody; i++)
        setJointAngles(i,jIndex);
    
    sMat Xj = sMat::Zero(); 
    if(model._jointTypes[0] == JointType::Fixed) {
        Xj = jointSpatialTransform(model._jointTypes[0],model._jointAxes[0], 0);
        _Xa[0] = Xj*model._Xtree[0];
    } else if(model._jointTypes[0] == JointType::Floating) {
        _Xa[0] = spatialTransform(_state.baseR, _state.basePosition);
    }

    for(int i : model._pathJoints[bodyId]) {
        Xj = jointSpatialTransform(model._jointTypes[i],model._jointAxes[i], model._jointAxisCoef[i]*model._q[i]);
        _Xa[i] = Xj*model._Xtree[i];
        _Xa[i] = _Xa[i]*_Xa[model._parents[i]];
    }

    
    RotMat E = rotationFromX(_Xa[bodyId]);
    Vec3 r = translationFromX(_Xa[bodyId]);
    return r+E.transpose()*pos;
}

/** Calculates Jacobian matrix (6x1) of point wrt world frame.
 * @param[in] bodyId Body index. It can be retrieved by getLinkID() or getBodyIdx().
 * @param[in] state Robots states.
 * @param[in] pos Desired point position wrt body frame. (By default it is set to zero)
 * @param[out] out Jacobian matrix (6x1) wrt world frame.
 */
Eigen::MatrixXd RigidBodyKinematics::bodyJacobian(const int& bodyId, const ModelState& state, const Vec3& pos) {
    Vec3 r = forwardKinematic(bodyId, state, pos);
    sMat Xbody = spatialTransform(RotMat::Identity(), -r);

    Eigen::MatrixXd bodyJac(6,model.nDof);
    bodyJac.setZero();

    if(model._jointTypes[0] == JointType::Floating) {
        bodyJac.block<6,6>(0,0) = (_Xa[0]*Xbody).inverse();
    }

    for(int i : model._pathJoints[bodyId]) {
        auto it = std::find(model._movalbeJoints.begin(), model._movalbeJoints.end(),i);
        int idx = 0;        
        if(model._jointTypes[0] == JointType::Floating)
            idx = std::distance(model._movalbeJoints.begin(),it) + 5;
        else
            idx = std::distance(model._movalbeJoints.begin(),it);

        if(model._jointTypes[i] != JointType::Fixed) {
            _S[i] = jointMotionSubspace(model._jointTypes[i], model._jointAxes[i], model._jointAxisCoef[i]);  
            bodyJac.col(idx) = (_Xa[i]*Xbody).inverse()*_S[i];
        }
    }
    return bodyJac;
}

/** Solves inverse kinematic for given body position.
 * @param[in] bodyId Body index. It can be retrieved by getLinkID() or getBodyIdx().
 * @param[in] desPos Desired body position.
 * @param[in] err_tol Error tolerance.
 * @param[in] max_iter Maximum number of iterations.
 * @param[out] out Joint angles in radian.
 */
Eigen::VectorXd RigidBodyKinematics::inverseKinematic(const std::vector<int>& bodyId, const std::vector<Vec3>& desPos, const double& err_tol, const int& max_iter) {
    
    int iter = 0;
    int idxCol = 0;
    Vec3 error = Vec3::Zero();

    if(model._jointTypes[0] == JointType::Floating)
        idxCol = 6;
    else
        idxCol = 0;

    stateIK.baseR = rotationFromX(_Xa[0]);
    stateIK.basePosition = translationFromX(_Xa[0]);
      
    for(int i=0; i<bodyId.size(); i++) {
        while(true) {            
            Eigen::MatrixXd J = bodyJacobian(bodyId[i], stateIK).block(3,idxCol,3,model.nDof-idxCol);
            Vec3 pos = forwardKinematic(bodyId[i], stateIK);
            error = desPos[i] - pos;
            stateIK.q = stateIK.q + J.completeOrthogonalDecomposition().pseudoInverse()*error;

            iter++;
            if(iter > max_iter) {
                std::cerr << "Error: Inverse kinematic could not be solved. Maximum iteration exeeded." << std::endl;
                std::cerr << "Error norm:" << error.norm() << std::endl;
                std::cerr << "Number of iteration:" << iter << std::endl;
                break;
            }

            if(error.norm() <= err_tol) {                
                break;
            }
        }
    }
    return stateIK.q;
}