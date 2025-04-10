#include "main.hpp"

int main(int argc, char** argv) {   
    /* #region: Raisim */
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\activation.raisim");
    raisim::World world;
    world.setGravity({0,0,0});

    world.setTimeStep(0.001);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 2, 0.15, 0.001, 2, 0.001);
    auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/rsc/go2/urdf/go2_2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0,0,0.7,1,0, 0, 0, 
                      0*M_PI/180, 30*M_PI/180, -60*M_PI/180, 
                     -0*M_PI/180, 30*M_PI/180, -60*M_PI/180, 
                      0*M_PI/180, 30*M_PI/180, -60*M_PI/180, 
                     -0*M_PI/180, 30*M_PI/180, -60*M_PI/180; 
    genVelocity << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Q_init.resize(12);
    Q_init = genCoordinates.tail(12);
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

        /* #region: Forward Kinematic */
        raisim::Vec<3> framePos;
        robot->getFramePosition(robot->getFrameIdxByName("FL_foot_joint"), framePos);
        std::cout << "Forward Kinematic Results" << std::endl;
        RSINFO(framePos.e())
        RSWARN(robotModel.forwardKinematic(robotModel.getFrameID("FL_foot_joint"), robotState))
        std::cout << "-----------------------" << std::endl;
        /* #endregion */
        
        /* #region: Jacobian Matrix */
        Eigen::MatrixXd jac(6,robot->getDOF()), jacT(3,robot->getDOF()), jacR(3,robot->getDOF());
        jacT.setZero(); jacR.setZero();
        robot->getDenseFrameJacobian(robot->getFrameIdxByName("FL_foot_joint"),jacT);
        robot->getDenseFrameRotationalJacobian(robot->getFrameIdxByName("FL_foot_joint"),jacR);
        jac << jacR, jacT;
        std::cout << "Jacobian Matrix Results" << std::endl;
        RSINFO(jac)
        RSWARN(robotModel.bodyJacobian(robotModel.getFrameID("FL_foot_joint"),robotState));
        std::cout << "-----------------------" << std::endl;
        /* #endregion */

        /* #region: Inverse Kinematic */
        Vec3 refPosFL = robotModel.forwardKinematic(robotModel.getFrameID("FL_foot_joint"), robotState);
        Vec3 refPosFR = robotModel.forwardKinematic(robotModel.getFrameID("FR_foot_joint"), robotState);
        Vec3 refPosRL = robotModel.forwardKinematic(robotModel.getFrameID("RL_foot_joint"), robotState);
        Vec3 refPosRR = robotModel.forwardKinematic(robotModel.getFrameID("RR_foot_joint"), robotState);
        Qik = robotModel.inverseKinematic({robotModel.getFrameID("FL_foot_joint"), 
                                           robotModel.getFrameID("FR_foot_joint"), 
                                           robotModel.getFrameID("RL_foot_joint"), 
                                           robotModel.getFrameID("RR_foot_joint")}, 
                                          {refPosFL, refPosFR, refPosRL, refPosRR}, Q_init, Vec3{0,0,0.7}, RotMat::Identity());
        Q_init = Qik;
        std::cout << "Inverse Kinematic Results" << std::endl;
        RSINFO(robotState.q)
        RSWARN(Qik)
        std::cout << "-----------------------" << std::endl;
        /* #endregion */
        

        /* #region: PD control */
        if(t>=5) {
            refQ << 0*M_PI/180, 0*M_PI/180, -0*M_PI/180,
                   -0*M_PI/180, 0*M_PI/180, -0*M_PI/180,
                    0*M_PI/180, 0*M_PI/180, -0*M_PI/180,
                   -0*M_PI/180, 0*M_PI/180, -0*M_PI/180;
        } else{
            refQ << 0*M_PI/180, 30*M_PI/180, -60*M_PI/180,
                   -0*M_PI/180, 30*M_PI/180, -60*M_PI/180,
                    0*M_PI/180, 30*M_PI/180, -60*M_PI/180,
                   -0*M_PI/180, 30*M_PI/180, -60*M_PI/180;
        }

        refdQ.setZero();
        Eigen::MatrixXd Kp(12,12), Kd(12,12);
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F <<  0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(12)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(12));
        robot->setGeneralizedForce(F);
        /* #endregion */

        server.integrateWorldThreadSafe();

    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}