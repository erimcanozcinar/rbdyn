#ifndef MAIN_HPP
#define MAIN_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include "Eigen/Dense"
#include <filesystem>
#include "RigidBodyModel.hpp"
#include <chrono>

ModelState robotState, stateFK;
ModelStateDerivative robotDState;
RigidBodyModel robotModel("/home/erim/rbdyn/examples/rsc/go2/urdf/go2_gazebo.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(19), genVelocity(18), F(18), quat(4); 
Eigen::VectorXd refQ(12), refdQ(12), jffTorques(18), TauJoint(18);

Eigen::VectorXd Qik, Q_init;

Eigen::Vector3d Fcon_FL, Pcon_FL;
Eigen::Vector3d Fcon_FR, Pcon_FR;
Eigen::Vector3d Fcon_RL, Pcon_RL;
Eigen::Vector3d Fcon_RR, Pcon_RR;

#endif