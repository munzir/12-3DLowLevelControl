/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <nlopt.hpp>

#include "MyWindow.hpp"

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

struct comOptParams {
  SkeletonPtr robot;
  Eigen::Matrix<double, 25, 1> qInit;
};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 25, 1> q(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 25, 1> mGrad = q-optParams->qInit;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5*pow((q-optParams->qInit).norm(), 2));
}

double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
    + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(wheelAxis_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return optParams->robot->getBodyNode(0)->getTransform().matrix()(2,0);
}

double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(heading_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  Eigen::Matrix<double, 4, 4> Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double heading = atan2(Tf(0,0), -Tf(1,0));
  optParams->robot->setPositions(optParams->qInit);
  Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double headingInit = atan2(Tf(0,0), -Tf(1,0));
  return heading-headingInit;
}


dart::dynamics::SkeletonPtr createKrang() {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang =
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/Krang/Krang.urdf");
  krang->setName("krang");

  // Initiale pose parameters
  /* double headingInit = 0; // Angle of heading direction from positive x-axis of the world frame: we call it psi in the rest of the code
  double qBaseInit = -M_PI/3;
  Eigen::Vector3d xyzInit;
  xyzInit << 0, 0, 0.28;
  double qLWheelInit = 0;
  double qRWheelInit = 0;
  double qWaistInit = -4*M_PI/3;
  double qTorsoInit = 0;
  double qKinectInit = 0;
  Eigen::Matrix<double, 7, 1> qLeftArmInit; 
  qLeftArmInit << 1.102, -0.589, 0.000, -1.339, 0.000, 0.3, 0.000;
  Eigen::Matrix<double, 7, 1> qRightArmInit;
  qRightArmInit << -1.102, 0.589, 0.000, 1.339, 0.000, 1.4, 0.000; */

  // Read initial pose from the file
  ifstream file("../defaultInit.txt");
  assert(file.is_open());
  char line [1024];
  file.getline(line, 1024);
  std::istringstream stream(line);
  Eigen::Matrix<double, 24, 1> initPoseParams; // heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
  size_t i = 0; double newDouble;
  while((i < 24) && (stream >> newDouble)) initPoseParams(i++) = newDouble;
  file.close();
  double headingInit; headingInit = initPoseParams(0);
  double qBaseInit; qBaseInit = initPoseParams(1);
  Eigen::Vector3d xyzInit; xyzInit << initPoseParams.segment(2,3);
  double qLWheelInit; qLWheelInit = initPoseParams(5);
  double qRWheelInit; qRWheelInit = initPoseParams(6);
  double qWaistInit; qWaistInit = initPoseParams(7);
  double qTorsoInit; qTorsoInit = initPoseParams(8);
  double qKinectInit; qKinectInit = initPoseParams(9);
  Eigen::Matrix<double, 7, 1> qLeftArmInit; qLeftArmInit << initPoseParams.segment(10, 7);
  Eigen::Matrix<double, 7, 1> qRightArmInit; qRightArmInit << initPoseParams.segment(17, 7);
  
  // Calculating the axis angle representation of orientation from headingInit and qBaseInit: 
  // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
  Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

  // Ensure CoM is right on top of wheel axis
  const int dof = (const int)krang->getNumDofs();
  comOptParams optParams;
  optParams.robot = krang;
  optParams.qInit << aa.angle()*aa.axis(), xyzInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, qLeftArmInit, qRightArmInit; 
  nlopt::opt opt(nlopt::LN_COBYLA, dof);
  std::vector<double> q_vec(dof);
  double minf;
  opt.set_min_objective(comOptFunc, &optParams);
  opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
  opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
  opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(10);
  opt.optimize(q_vec, minf);
  Eigen::Matrix<double, 25, 1> q(q_vec.data());
  
  // Initializing the configuration
  krang->setPositions(q); 

  return krang;
}

dart::dynamics::SkeletonPtr createFloor()
{
  dart::dynamics::SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  dart::dynamics::BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
//  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::dynamics::SkeletonPtr floor = createFloor();
  dart::dynamics::SkeletonPtr robot = createKrang();

  world->addSkeleton(floor); //add ground and robot to the world pointer
  world->addSkeleton(robot);

  // create and initialize the world
  //Eigen::Vector3d gravity(0.0,  -9.81, 0.0);
  //world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  // create a window and link it to the world
  MyWindow window(new Controller(robot, robot->getBodyNode("lGripper"), robot->getBodyNode("rGripper") ) );
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(960, 720, "Forward Simulation");
  glutMainLoop();

  return 0;
}
