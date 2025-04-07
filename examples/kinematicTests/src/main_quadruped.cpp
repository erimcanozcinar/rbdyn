#include "main_quadruped.hpp"

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
    auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/rsc/quadruped/urdf/quadruped2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0, 0, 0.75, 1, 0, 0, 0, 10*M_PI/180, 30*M_PI/180, -60*M_PI/180, -10*M_PI/180, -30*M_PI/180, 60*M_PI/180, -10*M_PI/180, 30*M_PI/180, -60*M_PI/180, 10*M_PI/180, -30*M_PI/180, 60*M_PI/180;
    genVelocity << 0, 0, 0, 0, 0, 0, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;
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


    while(true) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        /* #region: Contact definition */
        /* #endregion */

        /* #region: Robot states definiton */
        quat = robot->getGeneralizedCoordinate().e().block(3,0,4,1);
        robotState.basePosition = robot->getGeneralizedCoordinate().e().head(3);
        robotState.baseOrientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized();
        robotState.baseR = quat2RotMat(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
        robotState.baseVelocity << robot->getGeneralizedVelocity().e().block(3,0,3,1), robot->getGeneralizedVelocity().e().head(3);
        robotState.q = robot->getGeneralizedCoordinate().e().tail(12);
        robotState.dq = robot->getGeneralizedVelocity().e().tail(12);

        robotDState.dBasePosition = robot->getGeneralizedVelocity().e().head(3);
        robotDState.dBaseVelocity << robot->getGeneralizedAcceleration().e().block(3,0,3,1), robot->getGeneralizedAcceleration().e().head(3);
        robotDState.ddq = robot->getGeneralizedAcceleration().e().tail(12);
        /* #endregion */

       /* #region: Forward Kinematic Result */
        raisim::Vec<3> framePos;
        robot->getFramePosition(robot->getFrameIdxByName("footFrame_lf"), framePos);
        // std::cout << "Forward Kinematic Results" << std::endl;
        // RSINFO(framePos.e())
        // RSWARN(robotModel.forwardKinematic(robotModel.getBodyID("foot_lf"), robotState))
        // std::cout << "-----------------------" << std::endl;
        /* #endregion */
       
        /* #region: Jacobian Matrix Results  */
        Eigen::MatrixXd jac(6,robot->getDOF()), jacT(3,robot->getDOF()), jacR(3,robot->getDOF());
        jacT.setZero(); jacR.setZero();
        robot->getDenseFrameJacobian(robot->getFrameIdxByName("footFrame_rf"),jacT);
        robot->getDenseFrameRotationalJacobian(robot->getFrameIdxByName("footFrame_rf"),jacR);
        // std::cout << "Jacobian Matrix Results" << std::endl;
        // jac << jacR, jacT;
        // RSINFO(jac.block(0,6,6,6));
        // RSWARN(robotModel.bodyJacobian(robotModel.getBodyID("foot_rf"), robotState).block(0,6,6,6))
        // std::cout << "-----------------------" << std::endl;
        /* #endregion */    

        /* #region: Inverse Kinematic Result */
        Vec3 refPosLF, refPosRF;
        refPosLF = robotModel.forwardKinematic(robotModel.getBodyID("foot_lf"), robotState);
        refPosRF = robotModel.forwardKinematic(robotModel.getBodyID("foot_rf"), robotState);
        // std::cout << "Inverse Kinematic Results" << std::endl;
        // RSINFO(robotState.q.head(6))
        RSWARN(robotModel.inverseKinematic({robotModel.getBodyID("foot_lf"), robotModel.getBodyID("foot_rf")}, {refPosLF, refPosRF}, genCoordinates.tail(12)).head(6))
        // std::cout << "-----------------------" << std::endl;
        /* #endregion */    
        

        /* #region: PD control */
        refQ << 10*M_PI/180, 30*M_PI/180, -60*M_PI/180,
                    -10*M_PI/180, -30*M_PI/180, 60*M_PI/180,
                    -10*M_PI/180, 30*M_PI/180, -60*M_PI/180,
                    10*M_PI/180, -30*M_PI/180, 60*M_PI/180;
        refdQ.setZero();
        Eigen::MatrixXd Kp(12,12), Kd(12,12);
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F << 0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(12)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(12));
        robot->setGeneralizedForce(0*F);
        /* #endregion */

        server.integrateWorldThreadSafe();
    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}