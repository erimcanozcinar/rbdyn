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
RigidBodyDynamics Dyn("/home/erim/rbdyn/examples/quadruped/rsc/urdf/tekir3mesh_new_V2copy.urdf");

double t, dt;
Eigen::VectorXd genCoordinates(19), genVelocity(18), F(18), quat(4); 
Eigen::VectorXd refQ(12), refdQ(12), jffTorques(2), TauJoint(7);

raisim::Mat<3,3> frameOri;

Eigen::Vector3d Fcon, Pcon;

#endif