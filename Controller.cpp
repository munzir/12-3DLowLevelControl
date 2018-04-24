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

#include "Controller.hpp"
#include <nlopt.hpp>
#include <string>

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
  : mRobot(_robot),
    mLeftEndEffector(_LeftendEffector),
    mRightEndEffector(_RightendEffector)
   {
  assert(_robot != nullptr);
  assert(_LeftendEffector != nullptr);
  assert(_RightendEffector != nullptr);

  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;

  mForces.setZero(dof);
  std::cout << "[controller] DoF: 1 " << dof << std::endl;
  mKp.setZero();
  std::cout << "[controller] DoF: 2 " << dof << std::endl;
  mKv.setZero();
  std::cout << "[controller] DoF: 3 " << dof << std::endl;

  for (int i = 0; i < 3; ++i) {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;
  }
  std::cout << "[controller] DoF: 4 " << dof << std::endl;

  // Remove position limits
  /*
  for (int i = 6; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);*/
  std::cout << "[controller] DoF: 5 " << dof << std::endl;

  // Set joint damping
  /*for (int i = 6; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);*/
  std::cout << "[controller] DoF: 6 " << dof << std::endl;

}

//=========================================================================
Controller::~Controller() {}
//=========================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

//=========================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//========================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 18, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 18, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition) {
  /*using namespace dart;
  using namespace std;
  const int dof = (const int)mRobot->getNumDofs();
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
  double weightRight = 1.0, weightLeft = 1.0, weightRegulator = 1.0, weightBalance = 10.0;
  double KpzCOM = 750.0, KvzCOM = 250.0;

  // Left arm
  Eigen::Vector3d xLft    = mLeftEndEffector->getTransform().translation();
  Eigen::Vector3d dxLft   = mLeftEndEffector->getLinearVelocity();
  math::LinearJacobian JvLft   = mLeftEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJvLft  = mLeftEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::Vector3d ddxrefLft = -mKp*(xLft - _targetPosition) - mKv*dxLft;
  Eigen::Vector3d zeroColumn(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 7> zero7Columns;
  zero7Columns << zeroColumn, zeroColumn, zeroColumn, zeroColumn, \
              zeroColumn, zeroColumn, zeroColumn;
  Eigen::Matrix<double, 3, 18> FullJacobianLeft;
  FullJacobianLeft  << JvLft.block<3,3>(0,0), zeroColumn, JvLft.block<3,7>(0,3), zero7Columns;
  Eigen::Matrix<double, 3, 18> FullJacobianDerLft;
  FullJacobianDerLft  <<  dJvLft.block<3,3>(0,0), zeroColumn, dJvLft.block<3,7>(0,3), zero7Columns;
  Eigen::MatrixXd PLeft   = weightLeft*FullJacobianLeft;
  Eigen::VectorXd bLeft   = -weightLeft* ( FullJacobianDerLft*dq - ddxrefLft );

  // Right Arm
  Eigen::Vector3d xRgt    = mRightEndEffector->getTransform().translation();
  Eigen::Vector3d dxRgt   = mRightEndEffector->getLinearVelocity();
  math::LinearJacobian JvRgt   = mRightEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJvRgt  = mRightEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::Vector3d ddxrefRgt = -mKp*(xRgt - _targetPosition) - mKv*dxRgt;
  Eigen::Matrix<double, 3, 18> FullJacobianRight;
  FullJacobianRight <<  JvRgt.block<3,3>(0,0), zeroColumn, zero7Columns, JvRgt.block<3,7>(0,3);
  Eigen::Matrix<double, 3, 18> FullJacobianDerRgt;
  FullJacobianDerRgt  <<  dJvRgt.block<3,3>(0,0), zeroColumn, zero7Columns, dJvRgt.block<3,7>(0,3);
  Eigen::MatrixXd PRight  = weightRight*FullJacobianRight;
  Eigen::VectorXd bRight  = -weightRight*( FullJacobianDerRgt*dq - ddxrefRgt );

  // CoM
  double zCOM = mRobot->getCOM()(2);
  double dzCOM = mRobot->getCOMLinearVelocity()(2);
  Eigen::VectorXd JzCOM = mRobot->getCOMLinearJacobian().block<1,18>(2,0);
  Eigen::VectorXd dJzCOM = mRobot->getCOMLinearJacobianDeriv().block<1,18>(2,0);
  double ddzCOMref = -KpzCOM*zCOM - KvzCOM*dzCOM;
  Eigen::MatrixXd PBalance = weightBalance*JzCOM;
  double bBalance = -weightBalance*(dJzCOM.transpose()*dq - ddzCOMref);

  // Regulator
  Eigen::MatrixXd PRegulator = weightRegulator*Eigen::MatrixXd::Identity(dof, dof);
  Eigen::VectorXd bRegulator = -weightRegulator*10*dq;

  // Optimizer stuff
  OptParams optParams;
  Eigen::MatrixXd NewP(PRight.rows() + PLeft.rows() + PRegulator.rows() + PBalance.cols(), PRight.cols() );
  NewP << PRight,
          PLeft,
          PRegulator,
          PBalance.transpose();
  Eigen::VectorXd NewB(bRight.rows() + bLeft.rows() + bRegulator.rows() + 1, bRight.cols() );
  NewB << bRight,
          bLeft,
          bRegulator,
          bBalance;
  optParams.P = NewP;
  optParams.b = NewB;
  nlopt::opt opt(nlopt::LD_MMA, dof);
  std::vector<double> ddq_vec(dof);
  double minf;
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddq_vec, minf);
  Eigen::Matrix<double, 18, 1> ddq(ddq_vec.data());
 
  // Torques
  Eigen::MatrixXd M     = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  mForces = M*ddq + Cg;
  mRobot->setForces(mForces);*/
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//=========================================================================
dart::dynamics::BodyNode* Controller::getEndEffector(const std::string &s) const {
  if (s.compare("left")) {  return mLeftEndEffector; }
  else if (s.compare("right")) { return mRightEndEffector; }
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
