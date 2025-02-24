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
#include "Dynamics.hpp"

ModelState robotState;
ModelStateDerivative robotDState;
RigidBodyDynamics Dyn("/home/erim/rbdyn-main/examples/fixedBase/rsc/1dof.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(1), genVelocity(1), F(1), quat(4); 
Eigen::VectorXd refQ(1), refdQ(1), jffTorques(2), TauJoint(7);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif