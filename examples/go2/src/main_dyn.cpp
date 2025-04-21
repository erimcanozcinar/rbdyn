#include "main_dyn.hpp"

int main(int argc, char** argv) {   
    /* #region: Raisim */
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\activation.raisim");
    raisim::World world;
    // world.setGravity({0,0,0});
    robotModel.setGravity(world.getGravity().e());

    world.setTimeStep(0.001);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 2, 0.15, 0.001, 2, 0.001);
    auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto robot = world.addArticulatedSystem("/home/erim/rbdyn/examples/rsc/go2/urdf/go2_2.urdf");
    /* #endregion */
    
  
    /* #region: Initialize Robot */
    genCoordinates << 0,0,0.2965,1,0, 0, 0, 
                      0*M_PI/180, 51.9673*M_PI/180, -100.1530*M_PI/180, 
                     -0*M_PI/180, 51.9673*M_PI/180, -100.1530*M_PI/180, 
                      0*M_PI/180, 51.9673*M_PI/180, -100.1530*M_PI/180, 
                     -0*M_PI/180, 51.9673*M_PI/180, -100.1530*M_PI/180; 
    genVelocity << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Q_init.resize(12);
    refQ.resize(12);
    Q_init = genCoordinates.tail(12);
    robot->setGeneralizedCoordinate(genCoordinates);
    robot->setGeneralizedVelocity(genVelocity);
    /* #endregion */

    /* #region: Create Log file */
    FILE* Tau_rbdyn;
    FILE* Tau_rs1;
    FILE* test;
    Tau_rbdyn = fopen("/home/erim/rbdyn/examples/go2/Log/rbdynLog.txt", "w");
    Tau_rs1 = fopen("/home/erim/rbdyn/examples/go2/Log/rs1Log.txt", "w");
    test = fopen("/home/erim/rbdyn/examples/go2/Log/test.txt", "w");
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
    Vec6 Fex_FL, Fex_FR, Fex_RL, Fex_RR;

    int max_dur = 0;
    int iter = 0;
    int dur_sum = 0;
    
    while (1) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();

        /* #region: Contact definition */
        for (auto& contact : robot->getContacts()) // LF:3, RF:2, LB:1, RB:0
        {
            if (contact.skip()) continue;
            if (robot->getBodyIdx("FL_calf") == contact.getlocalBodyIndex())
            {
                Fcon_FL = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_FL = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("FR_calf") == contact.getlocalBodyIndex())
            {
                Fcon_FR = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_FR = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("RL_calf") == contact.getlocalBodyIndex())
            {
                Fcon_RL = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RL = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("RR_calf") == contact.getlocalBodyIndex())
            {
                Fcon_RR = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RR = contact.getPosition().e().transpose();
            }
        }
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

        /* #region: Inverse Kinematics */
        double hsin;
        if(t>3)
            hsin = 0.06*sin(2*M_PI*0.2*(t-3));
        else
            hsin = 0;

        auto start = std::chrono::high_resolution_clock().now();
        refQ = robotModel.inverseKinematic({robotModel.getFrameID("FL_foot_joint"),
                                     robotModel.getFrameID("FR_foot_joint"),
                                     robotModel.getFrameID("RL_foot_joint"),
                                     robotModel.getFrameID("RR_foot_joint")}, 
                                    {Vec3(0.1934,0.1420,0), Vec3(0.1934,-0.1420,0), Vec3(-0.1934,0.1420,0), Vec3(-0.1934,-0.1420,0)}, 
                                    Q_init, Vec3(0,0,0.2965+hsin), RotMat::Identity());
        auto end = std::chrono::high_resolution_clock().now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Execution time: " << duration.count() << " us" << std::endl;
        iter += 1;
        int dur = duration.count();
        dur_sum += dur;
        if(dur > max_dur)
            max_dur = dur;
        RSWARN(dur_sum/iter)
        /* #endregion */
                
        /* #region: RBDYN inverse dynamics*/
        Fex_FL << 0,0,0,Fcon_FL;
        Fex_FR << 0,0,0,Fcon_FR;
        Fex_RL << 0,0,0,Fcon_RL;
        Fex_RR << 0,0,0,Fcon_RR;
        robotModel.applyExternalForce(robotModel.getBodyID("FL_foot"), Fex_FL);
        robotModel.applyExternalForce(robotModel.getBodyID("FR_foot"), Fex_FR);
        robotModel.applyExternalForce(robotModel.getBodyID("RL_foot"), Fex_RL);
        robotModel.applyExternalForce(robotModel.getBodyID("RR_foot"), Fex_RR);

        // auto start = std::chrono::high_resolution_clock().now();
        jffTorques = robotModel.inverseDynamics(robotState,robotDState);
        // auto end = std::chrono::high_resolution_clock().now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // std::cout << "Execution time: " << duration.count() << " us" << std::endl;
        // iter += 1;
        // int dur = duration.count();
        // dur_sum += dur;
        // if(dur > max_dur)
        //     max_dur = dur;
        // RSWARN(dur_sum/iter)        
        /* #endregion */
       
        
        /* #region: Rasim inverse dynamics */
        for (int j=0; j<robot->getDOF()-6; j++) {
            axes[j] = robot->getJointAxis(j+1).e();
        }
            
        torqueFromInverseDynamics.head(3) = robot->getForceAtJointInWorldFrame(0).e();
        torqueFromInverseDynamics.segment<3>(3) = robot->getTorqueAtJointInWorldFrame(0).e();

        for (size_t j=1; j<robot->getDOF()-5; j++){
            torqueFromInverseDynamics(j+5) = (robot->getTorqueAtJointInWorldFrame(j).e()).dot(axes[j-1]);
        }
        /* #endregion */

             

        /* #region: PD control */
        refdQ.setZero();
        Eigen::MatrixXd Kp(12,12), Kd(12,12);
        Kp.setIdentity(); Kd.setIdentity();
        Kp = 9*Kp;
        Kd = 0.9*Kd;
        F <<  0, 0, 0, 0, 0, 0, Kp*(refQ - robot->getGeneralizedCoordinate().e().tail(12)) + Kd*(refdQ - robot->getGeneralizedVelocity().e().tail(12) + jffTorques.tail(12));
        robot->setGeneralizedForce(F);
        /* #endregion */

        server.integrateWorldThreadSafe();

        fprintf(test, "%f %i\n", t, dur);
        fprintf(Tau_rbdyn, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, jffTorques(0), jffTorques(1), jffTorques(2), jffTorques(3), jffTorques(4), jffTorques(5), jffTorques(6), jffTorques(7), jffTorques(8), jffTorques(9), jffTorques(10), jffTorques(11), jffTorques(12), jffTorques(13), jffTorques(14), jffTorques(15), jffTorques(16), jffTorques(17));   
        fprintf(Tau_rs1, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, torqueFromInverseDynamics(0), torqueFromInverseDynamics(1), torqueFromInverseDynamics(2), torqueFromInverseDynamics(3), torqueFromInverseDynamics(4), torqueFromInverseDynamics(5), torqueFromInverseDynamics(6), torqueFromInverseDynamics(7), torqueFromInverseDynamics(8), torqueFromInverseDynamics(9), torqueFromInverseDynamics(10), torqueFromInverseDynamics(11), torqueFromInverseDynamics(12), torqueFromInverseDynamics(13), torqueFromInverseDynamics(14), torqueFromInverseDynamics(15), torqueFromInverseDynamics(16), torqueFromInverseDynamics(17));
    }
    server.killServer();
    std::cout << "end of simulation" << std::endl;
}