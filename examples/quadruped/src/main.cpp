#include "main.hpp"

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

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/quadruped/rsc/urdf/quadruped2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0, 0, 0.75, 1, 0, 0, 0, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;
    genVelocity << 0, 0, 0, 0, 0, 0, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180, 0*M_PI/180;
    robot->setGeneralizedCoordinate(genCoordinates);
    robot->setGeneralizedVelocity(genVelocity);
    /* #endregion */

    /* #region: Create Log file */
    FILE* Tau_rbdyn;
    FILE* Tau_rs1;
    FILE* Tau_rs2;
    Tau_rbdyn = fopen("/home/erim/rbdyn/examples/quadruped/Log/rbdynLog.txt", "w");
    Tau_rs1 = fopen("/home/erim/rbdyn/examples/quadruped/Log/rs1Log.txt", "w");
    Tau_rs2 = fopen("/home/erim/rbdyn/examples/quadruped/Log/rs2Log.txt", "w");
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

    
    std::cout << robot->getBodyIdx("torso") << std::endl;

    // Fcon.setZero(); Pcon.setZero();
    // for (int loop = 0; loop < 5150; loop++) {
    while(true) {
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
        robotModel.inverseDynamics(robotState, robotDState);
        jffTorques = robotModel.genForce;
        
        /* #endregion */

        /* #region: PD control */
        if(t>=5)
            refQ << 10*M_PI/180*sin(2*M_PI*1*t), 30*M_PI/180*sin(2*M_PI*1*t), -60*M_PI/180*sin(2*M_PI*1*t),
                    -10*M_PI/180*sin(2*M_PI*1*t), -30*M_PI/180*sin(2*M_PI*1*t), 60*M_PI/180*sin(2*M_PI*1*t),
                    -10*M_PI/180*sin(2*M_PI*1*t), 30*M_PI/180*sin(2*M_PI*1*t), -60*M_PI/180*sin(2*M_PI*1*t),
                    10*M_PI/180*sin(2*M_PI*1*t), -30*M_PI/180*sin(2*M_PI*1*t), 60*M_PI/180*sin(2*M_PI*1*t);
        else
            refQ.setZero();

        refdQ.setZero();
        Eigen::MatrixXd Kp(12,12), Kd(12,12);
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 50*Kp;
        Kd = 3*Kd;
        F << 0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(12)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(12));
        robot->setGeneralizedForce(F);
        /* #endregion */
        robotModel.applyExternalForce(robot->getBodyIdx("torso"), Vec3(0,0,0), Vec6(0,0,0,10,0,0));
        robot->setExternalForce(robot->getBodyIdx("torso"),{0,0,0},{10,0,0});

        server.integrateWorldThreadSafe();

        
        

        fprintf(Tau_rbdyn, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, jffTorques(0), jffTorques(1), jffTorques(2), jffTorques(3), jffTorques(4), jffTorques(5), jffTorques(6), jffTorques(7), jffTorques(8), jffTorques(9), jffTorques(10), jffTorques(11), jffTorques(12), jffTorques(13), jffTorques(14), jffTorques(15), jffTorques(16), jffTorques(17));   
        fprintf(Tau_rs1, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, torqueFromInverseDynamics(0), torqueFromInverseDynamics(1), torqueFromInverseDynamics(2), torqueFromInverseDynamics(3), torqueFromInverseDynamics(4), torqueFromInverseDynamics(5), torqueFromInverseDynamics(6), torqueFromInverseDynamics(7), torqueFromInverseDynamics(8), torqueFromInverseDynamics(9), torqueFromInverseDynamics(10), torqueFromInverseDynamics(11), torqueFromInverseDynamics(12), torqueFromInverseDynamics(13), torqueFromInverseDynamics(14), torqueFromInverseDynamics(15), torqueFromInverseDynamics(16), torqueFromInverseDynamics(17));   
        fprintf(Tau_rs2, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, TauJoint(0), TauJoint(1), TauJoint(2), TauJoint(3), TauJoint(4), TauJoint(5), TauJoint(6), TauJoint(7), TauJoint(8), TauJoint(9), TauJoint(10), TauJoint(11), TauJoint(12), TauJoint(13), TauJoint(14), TauJoint(15), TauJoint(16), TauJoint(17));   
    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}