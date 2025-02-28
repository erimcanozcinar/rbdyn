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
RigidBodyDynamics Dyn("/home/erim/rbdyn/examples/floatingBase/rsc/1dof_float2.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(10), genVelocity(9), F(9), quat(4); 
Eigen::VectorXd refQ(3), refdQ(3), jffTorques(3), TauJoint(9);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif