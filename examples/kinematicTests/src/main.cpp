#include "main.hpp"

int main(int argc, char** argv) {   
    /* #region: Raisim */
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\activation.raisim");
    raisim::World world;
    // world.setGravity({0,0,0});

    world.setTimeStep(0.001);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 2, 0.15, 0.001, 2, 0.001);
    auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/rsc/fixedBase/1dof2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180; 
    genVelocity << 0, 0, 0, 0;
    robot->setGeneralizedCoordinate(genCoordinates);
    robot->setGeneralizedVelocity(genVelocity);
    /* #endregion */

    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.setMap("default");
    server.focusOn(robot);
    server.launchServer();
    raisim::MSLEEP(5000);
    /* #endregion */
    
    while (1) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        /* #region: Contact definition */
        for (auto& contact : robot->getContacts()) // LF:3, RF:2, LB:1, RB:0
        {
            if (contact.skip()) continue;
            if (robot->getBodyIdx("link2") == contact.getlocalBodyIndex())
            {
                Fcon = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon = contact.getPosition().e().transpose();
            }
        }
        /* #endregion */

        /* #region: Robot states definiton */
        quat = robot->getGeneralizedCoordinate().e().block(3,0,4,1);
        robotState.basePosition << 0, 0, 0.15;
        robotState.baseOrientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized();
        robotState.baseR = quat2RotMat(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
        robotState.baseVelocity = Vec6::Zero();
        robotState.q = robot->getGeneralizedCoordinate().e();
        robotState.dq = robot->getGeneralizedVelocity().e();

        robotDState.dBasePosition = Eigen::Vector3d::Zero();
        robotDState.dBaseVelocity << Vec6::Zero();
        robotDState.ddq = robot->getGeneralizedAcceleration().e();
        /* #endregion */

        /* #region: Forward Kinematic */
        raisim::Vec<3> framePos;
        robot->getFramePosition(robot->getFrameIdxByName("Joint6"), framePos);
        std::cout << "Forward Kinematic Results" << std::endl;
        RSINFO(framePos.e())
        RSWARN(robotModel.forwardKinematic(robotModel.getJointID("Joint6"), robotState))
        std::cout << "-----------------------" << std::endl;
        /* #endregion */
        
        /* #region: Jacobian Matrix */
        Eigen::MatrixXd jac(6,robot->getDOF()), jacT(3,robot->getDOF()), jacR(3,robot->getDOF());
        jacT.setZero(); jacR.setZero();
        robot->getDenseFrameJacobian(robot->getFrameIdxByName("Joint6"),jacT);
        robot->getDenseFrameRotationalJacobian(robot->getFrameIdxByName("Joint6"),jacR);
        jac << jacR, jacT;
        std::cout << "Jacobian Matrix Results" << std::endl;
        RSINFO(jac)
        RSWARN(robotModel.bodyJacobian(robotModel.getJointID("Joint6"),robotState));
        std::cout << "-----------------------" << std::endl;
        /* #endregion */

        /* #region: Inverse Kinematic */
        Vec3 refPosL, refPosR;
        refPosL = robotModel.forwardKinematic(robotModel.getJointID("Joint3"), robotState);
        refPosR = robotModel.forwardKinematic(robotModel.getJointID("Joint6"), robotState);
        Qik = robotModel.inverseKinematic({robotModel.getJointID("Joint3"), robotModel.getJointID("Joint6")}, {refPosL, refPosR}, genCoordinates, robotState.basePosition, robotState.baseR);
        std::cout << "Inverse Kinematic Results" << std::endl;
        RSINFO(robotState.q)
        RSWARN(Qik)
        std::cout << "-----------------------" << std::endl;
        /* #endregion */
        

        /* #region: PD control */
        if(t>=5)
            refQ << 20*M_PI/180, -10*M_PI/180, 10*M_PI/180, -10*M_PI/180;
        else
            refQ << 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;

        refdQ << 0, 0;
        Eigen::Matrix4d Kp, Kd;
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F <<  Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(4)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(4));
        robot->setGeneralizedForce(F);
        /* #endregion */

        server.integrateWorldThreadSafe();

    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}