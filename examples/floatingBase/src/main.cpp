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

    auto robot = world.addArticulatedSystem("/home/erim/RaiSim_Simulations/invDyn_test/invDyn_test/rsc/1dof_2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0, 0, 0.15, 1, 0, 0, 0, 0*M_PI/180, 0*M_PI/180;
    genVelocity << 0, 0, 0, 0, 0, 0, 0*M_PI/180, 0*M_PI/180;
    robot->setGeneralizedCoordinate(genCoordinates);
    robot->setGeneralizedVelocity(genVelocity);
    /* #endregion */

    /* #region: Create Log file */
    FILE* fp0;
    fp0 = fopen("/home/erim/RaiSim_Simulations/invDyn_test/invDyn_test/Log/dataLog.txt", "w");
    /* #endregion */

    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.setMap("default");
    server.focusOn(robot);
    server.launchServer();
    raisim::MSLEEP(5000);
    /* #endregion */

    robot->setComputeInverseDynamics(true);
    Eigen::VectorXd torqueFromInverseDynamics(robot->getDOF());
    std::vector<Eigen::Vector3d> axes(robot->getDOF());
    torqueFromInverseDynamics.setZero();

    // fbDyn.init();
    Dyn.init(world.getGravity().e());

    // Fcon.setZero(); Pcon.setZero();
    for (int loop = 0; loop < 5050; loop++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        // for (auto& contact : robot->getContacts()) // LF:3, RF:2, LB:1, RB:0
        // {
        //     if (contact.skip()) continue;
        //     if (robot->getBodyIdx("link") == contact.getlocalBodyIndex())
        //     {
        //         Fcon = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
        //         Pcon = contact.getPosition().e().transpose();
        //     }
        // }

        Eigen::Vector3d ypr;
        ypr = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized().toRotationMatrix().eulerAngles(2,1,0);
        quat = robot->getGeneralizedCoordinate().e().block(3,0,4,1);

        robotState.basePosition = robot->getGeneralizedCoordinate().e().head(3);
        robotState.baseOrientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized();
        robotState.baseR = quat2RotMat(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
        robotState.baseVelocity << robot->getGeneralizedVelocity().e().block(3,0,3,1), robot->getGeneralizedVelocity().e().head(3);
        robotState.q = robot->getGeneralizedCoordinate().e().tail(2);
        robotState.dq = robot->getGeneralizedVelocity().e().tail(2);

        robotDState.dBasePosition = robot->getGeneralizedVelocity().e().head(3);
        robotDState.dBaseVelocity << robot->getGeneralizedAcceleration().e().block(3,0,3,1), robot->getGeneralizedAcceleration().e().head(3);
        robotDState.ddq = robot->getGeneralizedAcceleration().e().tail(2);
        
        /* #region: RaiSim Inverse Dynamics */
        for (int j=0; j<robot->getDOF()-6; j++) {
            axes[j] = robot->getJointAxis(j+1).e();
        }

        torqueFromInverseDynamics.head(3) = robot->getForceAtJointInWorldFrame(0).e();
        torqueFromInverseDynamics.segment<3>(3) = robot->getTorqueAtJointInWorldFrame(0).e();

        for (size_t j=1; j<robot->getDOF()-5; j++){
            torqueFromInverseDynamics(j+5) = (robot->getTorqueAtJointInWorldFrame(j).e()).dot(axes[j-1]);
        }
            
        TauJoint = robot->getMassMatrix().e()*robot->getUdot().e() + robot->getNonlinearities(world.getGravity()).e();
        RSINFO(TauJoint)       
        RSWARN(torqueFromInverseDynamics)
        // RSINFO(fbDyn.inverseDynamics(robotState, robotDState));
        // Eigen::VectorXd a = fbDyn.inverseDynamics(robotState, robotDState);
        Dyn.inverseDynamics(robotState, robotDState);
        /* #endregion */

        /* #region: PD control */
        if(t>=5)
            refQ << 40*M_PI/180, -20*M_PI/180;
        else
            refQ << 0*M_PI/180, 0*M_PI/180;

        refdQ << 0, 0;
        Eigen::Matrix2d Kp, Kd;
        Kp << 50, 0, 0, 50;
        Kd << 3, 0, 0, 3;
        F << 0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(2)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(2));
        robot->setGeneralizedForce(F);
        /* #endregion */

        server.integrateWorldThreadSafe();

        // fprintf(fp0, "%f %f %f %f\n", t, jffTorques(0), torqueFromInverseDynamics(6), TauJoint(6));   
    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}