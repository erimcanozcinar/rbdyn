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
RigidBodyModel robotModel("/home/erim/rbdyn/examples/fixedBase/rsc/1dof2.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(4), genVelocity(4), F(4), quat(4); 
Eigen::VectorXd refQ(4), refdQ(4), jffTorques(4), TauJoint(7);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif