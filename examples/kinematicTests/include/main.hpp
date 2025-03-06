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

ModelState robotState;
ModelStateDerivative robotDState;
RigidBodyModel robotModel("/home/erim/rbdyn/examples/kinematicTests/rsc/1dof.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(2), genVelocity(2), F(2), quat(4); 
Eigen::VectorXd refQ(2), refdQ(2), jffTorques(2), TauJoint(2);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif