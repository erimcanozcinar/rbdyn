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
RigidBodyModel robotModel("/home/erim/rbdyn/examples/rsc/floatingBase/1dof_float2.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(11), genVelocity(10), F(10), quat(4); 
Eigen::VectorXd refQ(4), refdQ(4), jffTorques(10), TauJoint(10);

Eigen::VectorXd Qik;

Eigen::Vector3d Fcon, Pcon;

#endif