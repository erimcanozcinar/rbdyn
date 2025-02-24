#ifndef MAIN2_HPP
#define MAIN2_HPP

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
RigidBodyDynamics Dyn("/home/erim/rbdyn/examples/floatingBase/rsc/1dof_2.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(9), genVelocity(8), F(8), quat(4); 
Eigen::VectorXd refQ(1), refdQ(1), jffTorques(2), TauJoint(7);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif