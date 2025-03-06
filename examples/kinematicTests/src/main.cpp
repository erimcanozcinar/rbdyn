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

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/kinematicTests/rsc/1dof.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0*M_PI/180, 0*M_PI/180; 
    genVelocity << 0, 0, 0, 0;
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

    robot->setComputeInverseDynamics(true);
    Eigen::VectorXd torqueFromInverseDynamics(robot->getDOF());
    std::vector<Eigen::Vector3d> axes(robot->getDOF());
    torqueFromInverseDynamics.setZero();

    
    while (1) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        for (auto& contact : robot->getContacts()) // LF:3, RF:2, LB:1, RB:0
        {
            if (contact.skip()) continue;
            if (robot->getBodyIdx("link2") == contact.getlocalBodyIndex())
            {
                Fcon = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon = contact.getPosition().e().transpose();
            }
        }

        Eigen::Vector3d ypr;
        ypr = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized().toRotationMatrix().eulerAngles(2,1,0);
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

        /* #region: Kinematis */
        robotModel.forwardKinematics(robotState);
        /* #endregion */
     

        /* #region: RaiSim Inverse Dynamics */
        for (int j=0; j<robot->getDOF(); j++) {
            axes[j] = robot->getJointAxis(j+1).e();
        }

        for (size_t j=0; j<robot->getDOF(); j++){
            robot->getFrameOrientation(j, frameOri);
            torqueFromInverseDynamics(j) = (robot->getTorqueAtJointInWorldFrame(j+1).e()).dot(axes[j]);
        }
            
        TauJoint = robot->getMassMatrix().e()*robot->getUdot().e() + robot->getNonlinearities(world.getGravity()).e();
        // RSWARN(TauJoint)       
        // RSWARN(torqueFromInverseDynamics)
        // robotModel.inverseDynamics(robotState, robotDState);
        /* #endregion */

        /* #region: PD control */
        if(t>=5)
            refQ << 10*M_PI/180, -10*M_PI/180;
        else
            refQ << 0*M_PI/180, 0*M_PI/180;

        refdQ << 0, 0;
        Eigen::Matrix2d Kp, Kd;
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F <<  Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(2)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(2));
        robot->setGeneralizedForce(F);
        /* #endregion */
        // robotModel.applyExternalForce(2, Vec3(0.01,0,0.25), Vec6(0,0,0,Fcon(0),Fcon(1),Fcon(2)));
        // robotModel.applyExternalForce(robotModel.getBodyID("link1"), Vec3(0,0,0.125), Vec6(0,0,0,10,0,0));
        // robot->setExternalForce(robot->getBodyIdx("link1"),{10,0,0});
        // robot->setExternalTorque(robot->getBodyIdx("link2"), {10,0,0});

        server.integrateWorldThreadSafe();

    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}