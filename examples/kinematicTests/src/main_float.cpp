#include "main_float.hpp"

int main(int argc, char** argv) {   
    /* #region: Raisim */
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\activation.raisim");
    raisim::World world;
    world.setGravity({0,0,0});
    robotModel.setGravity(world.getGravity().e());

    world.setTimeStep(0.001);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 2, 0.15, 0.001, 2, 0.001);
    // auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/rsc/floatingBase/1dof_float2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0, 0, 0.15, 1, 0, 0, 0, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;
    genVelocity << 0, 0, 0, 0, 0, 0, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;
    robot->setGeneralizedCoordinate(genCoordinates);
    robot->setGeneralizedVelocity(genVelocity);
    /* #endregion */

    /* #region: Create Log file */
    /* #endregion */

    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.setMap("default");
    server.focusOn(robot);
    server.launchServer();
    raisim::MSLEEP(5000);
    /* #endregion */

    // Fcon.setZero(); Pcon.setZero();
    // for (int loop = 0; loop < 5150; loop++) {
    while(true) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        /* #region: State definition */
        quat = robot->getGeneralizedCoordinate().e().block(3,0,4,1);

        robotState.basePosition = robot->getGeneralizedCoordinate().e().head(3);
        robotState.baseOrientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized();
        robotState.baseR = quat2RotMat(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
        robotState.baseVelocity << robot->getGeneralizedVelocity().e().block(3,0,3,1), robot->getGeneralizedVelocity().e().head(3);
        robotState.q = robot->getGeneralizedCoordinate().e().tail(4);
        robotState.dq = robot->getGeneralizedVelocity().e().tail(4);

        robotDState.dBasePosition = robot->getGeneralizedVelocity().e().head(3);
        robotDState.dBaseVelocity << robot->getGeneralizedAcceleration().e().block(3,0,3,1), robot->getGeneralizedAcceleration().e().head(3);
        robotDState.ddq = robot->getGeneralizedAcceleration().e().tail(4);
        /* #endregion */
        
        /* #region: Forward Kinematic */
        raisim::Vec<3> framePos;
        robot->getFramePosition(robot->getFrameIdxByName("Joint6"), framePos);
        std::cout << "Forward Kinematic Results" << std::endl;
        RSINFO(framePos.e())
        RSWARN(robotModel.forwardKinematic(robotModel.getFrameID("Joint6"), robotState))
        std::cout << "-----------------------" << std::endl;
        /* #endregion */
        
        /* #region: Jacobian Matrix */
        Eigen::MatrixXd jac(6,robot->getDOF()), jacT(3,robot->getDOF()), jacR(3,robot->getDOF());
        jacT.setZero(); jacR.setZero();
        robot->getDenseFrameJacobian(robot->getFrameIdxByName("Joint3"),jacT);
        robot->getDenseFrameRotationalJacobian(robot->getFrameIdxByName("Joint3"),jacR);
        jac << jacR, jacT;
        std::cout << "Jacobian Matrix Results" << std::endl;
        RSINFO(jac)
        RSWARN(robotModel.bodyJacobian(robotModel.getFrameID("Joint3"),robotState));
        std::cout << "-----------------------" << std::endl;
        /* #endregion */

        /* #region: Inverse Kinematic */
        Vec3 refPosL, refPosR;
        refPosL = robotModel.forwardKinematic(robotModel.getFrameID("Joint3"), robotState);
        refPosR = robotModel.forwardKinematic(robotModel.getFrameID("Joint6"), robotState);
        Qik = robotModel.inverseKinematic({robotModel.getFrameID("Joint3"), robotModel.getFrameID("Joint6")}, {refPosL, refPosR}, genCoordinates.tail(4), robotState.basePosition, robotState.baseR);
        std::cout << "Inverse Kinematic Results" << std::endl;
        RSINFO(robotState.q)
        RSWARN(Qik)
        std::cout << "-----------------------" << std::endl;
        /* #endregion */

        /* #region: PD control */
        if(t>=5)
            refQ << 10*M_PI/180, -20*M_PI/180, -15*M_PI/180, 5*M_PI/180;
        else
            refQ << 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;

        refdQ << 0, 0, 0, 0;
        Eigen::Matrix4d Kp, Kd;
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F << 0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(4)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(4));
        robot->setGeneralizedForce(F);
        /* #endregion */
        // Dyn.applyExternalForce(2, Vec3(0,0,0.25), Vec6(0,0,0,10,0,0));
        // robot->setExternalForce(robot->getBodyIdx("link2"),{0,0,0.25},{10,0,0});

        server.integrateWorldThreadSafe();

    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}