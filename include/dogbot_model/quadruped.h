#ifndef QUADRUPEDMODEL
#define QUADRUPEDMODEL

#include <cstdlib>
#include <iostream>
#include <cmath>
// Eigen headers 
#include <Eigen/Core>
#include "dogbot_model/utils.h"
// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include "dogbot_model/alglib/optimization.h"
#include "dogbot_model/kdl_object.h"
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/conversions.hpp>

class QUADRUPED
{
  public:
  
   enum SWING_LEGS { L1, L2, L3}; // 4 legs, br-fl, bl-fr
    enum SWING_LEG { BR, FL, BL, FR}; // 4 legs, br-fl, bl-fr
  //robot
   QUADRUPED();
   QUADRUPED(std::string modelFile);
  
  // update the robot state
	void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,18,1> &eigenJointPos, Eigen::Matrix<double,18,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			
  
  // Solve quadratic problem for contact forces
Eigen::MatrixXd qpproblem( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::MatrixXd vdotswdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot);

  Eigen::VectorXd qpproblembr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::VectorXd vdotswdes,  SWING_LEGS swinglegs, Eigen::MatrixXd vdotobjdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot);
  Eigen::VectorXd qpproblemol( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes,  SWING_LEG swingleg, Eigen::MatrixXd vdotobjdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot);

  // get function
	Eigen::VectorXd getBiasMatrix();
	Eigen::VectorXd getGravityMatrix();
	Eigen::MatrixXd getMassMatrix();
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getBiasAcc();
  Eigen::MatrixXd getTransMatrix();
  Eigen::VectorXd getBiasMatrixCOM();
	Eigen::VectorXd getGravityMatrixCOM();
	Eigen::MatrixXd getMassMatrixCOM();
  Eigen::MatrixXd getMassMatrixCOM_com();
  Eigen::MatrixXd getMassMatrixCOM_joints();
  Eigen::MatrixXd getJacobianCOM();
  Eigen::MatrixXd getJacobianCOM_linear();
  
	Eigen::MatrixXd getBiasAccCOM();
  Eigen::MatrixXd getBiasAccCOM_linear();
  Eigen::MatrixXd getBRpos();
  Eigen::MatrixXd getBLpos();
  Eigen::MatrixXd getFLpos();
  Eigen::MatrixXd getFRpos();
  Eigen::MatrixXd getBRvel();
  Eigen::MatrixXd getBLvel();
  Eigen::MatrixXd getFLvel();
  Eigen::MatrixXd getFRvel();
  Eigen::Matrix<double,3,3> getBRworldtransform();
  Eigen::Matrix<double,3,3> getBLworldtransform();
  Eigen::Matrix<double,3,3> getFLworldtransform();
  Eigen::Matrix<double,3,3> getFRworldtransform();
  Eigen::MatrixXd getCOMpos();
  Eigen::MatrixXd getCOMvel();
  Eigen::MatrixXd getJobj();
  Eigen::MatrixXd getLambdaObj();
  Eigen::MatrixXd getBiasObj();
  Eigen::MatrixXd getJObj();
   Eigen::MatrixXd getRotwrench();
  Eigen::MatrixXd getOBJpos();
  Eigen::MatrixXd getOBJvel();
Eigen::MatrixXd getsolution();
  void addObj(KDLObject &_obj);
KDL::Frame getEEFrame();
// contacts
    std::vector<Contact> getContacts() const;
    void setContacts(const std::vector<Contact> &contacts);
  
  double getMass();
  int getDoFsnumber();

  private:
  
  // int for DoFs number
  unsigned int n;
  
  // Total mass of the robot
  double robot_mass;

  // KinDynComputations element
  iDynTree::KinDynComputations kinDynComp;
  
  // world to floating base transformation
  iDynTree::Transform world_H_base;
  
  // Joint position
  iDynTree::VectorDynSize jointPos;
  
  // Floating base velocity
  iDynTree::Twist         baseVel;
  
  // Joint velocity
  iDynTree::VectorDynSize jointVel;
  
  // Gravity acceleration
  iDynTree::Vector3       gravity; 


// Position vector base+joints
  iDynTree::VectorDynSize  qb;

  // Velocity vector base+joints
  iDynTree::VectorDynSize  dqb;

  // Position vector COM+joints
  iDynTree::VectorDynSize  q;

  // Velocity vector COM+joints
  iDynTree::VectorDynSize  dq;

  // Joints limit vector

  iDynTree::VectorDynSize  qmin;
  iDynTree::VectorDynSize  qmax;
  
  // Center of Mass Position

  iDynTree::Vector6 CoM;

  iDynTree::Vector6 xobj;
  // Center of mass velocity

  iDynTree::Vector6 CoM_vel;
  Eigen::VectorXd x_eigen;
  iDynTree::Vector6 xobjvel;
  //Mass matrix
  iDynTree::FreeFloatingMassMatrix MassMatrix;
  
  //Bias Matrix
  iDynTree::VectorDynSize Bias;
  
  //Gravity Matrix
  iDynTree::MatrixDynSize GravMatrix;
  
  // Jacobian
  iDynTree::MatrixDynSize Jac;

   // Jacobian
  iDynTree::MatrixDynSize Jac_lin;

   // Jacobian derivative
  iDynTree::MatrixDynSize JacDot;

  // Jacobian derivative
  iDynTree::MatrixDynSize JacDotObj;
  
  //CoM Jacobian
  iDynTree::MatrixDynSize Jcom;
  
  // Bias acceleration J_dot*q_dot
  iDynTree::MatrixDynSize Jdqd;
  
  // Transformation Matrix
  iDynTree::MatrixDynSize T;
  
  iDynTree::MatrixDynSize Rotwrench;
  // Transformation matrix time derivative
  iDynTree::MatrixDynSize T_inv_dot;
  
  //Model
  iDynTree::Model model;
  iDynTree::ModelLoader mdlLoader;
  
   //Mass matrix in CoM representation
  iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
  
  
  //Bias Matrix in CoM representation
  iDynTree::VectorDynSize BiasCOM;
  
  //Gravity Matrix in CoM representation
  iDynTree::MatrixDynSize GravMatrixCOM;
  
  // Jacobian in CoM representation
  iDynTree::MatrixDynSize JacCOM;

  //Jacobian in CoM representation (only linear part)
  iDynTree::MatrixDynSize JacCOM_lin;

  // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM;

   // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM_lin;
  


    //Mass matrix in CoM representation
  iDynTree::MatrixDynSize LambdaObj;

   //Bias Matrix in CoM representation
  iDynTree::VectorDynSize BiasObj;

  //CoM Jacobian
  iDynTree::MatrixDynSize Jobj;

  //CoM Jacobian
  iDynTree::MatrixDynSize Jobjdot;

  
  //Create the robot
  void createrobot(std::string modelFile);
  
  // Compute the Jacobian
  void  computeJac();

  void  ComputeJaclinear();
  // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
  void computeTransformation(Eigen::VectorXd Vel_);
  
  // Compute bias acceleration J_dot*q_dot
  void computeJacDotQDot();
  
  void computeJdqdCOMlinear();

  void ComputeJacstlinear();
  // Compute Jacobian time derivative
  void computeJacDot(Eigen::Matrix<double, 24,1> Vel_);
  
  void  computeJacDotObj(Eigen::Matrix<double,24,1> Vel_);
  //Compute partial derivative
  Eigen::VectorXd  getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx);

// end-effector
    KDL::Frame f_F_ee_;             // end-effector frame in flange frame
    KDL::Frame s_F_ee_;             // end-effector frame in spatial frame
    KDL::Twist s_V_ee_;             // end-effector twist in spatial frame
    KDL::Jacobian s_J_ee_;          // end-effector Jacobian in spatial frame
    KDL::Jacobian b_J_ee_;          // end-effector Jacobian in body frame
    KDL::Jacobian s_J_dot_ee_;      // end-effector Jacobian dot in spatial frame
    KDL::Jacobian b_J_dot_ee_;      // end-effector Jacobian dot in body frame
    KDL::Twist s_J_dot_q_dot_ee_;   // end-effector Jdot*qdot in spatial frame


    //object
    KDLObject* obj_;
    KDL::Frame ee_F_obj_;           // object frame in end-effector
    KDL::Jacobian s_J_obj_;         // object Jacobian in spatial frame
    KDL::Jacobian b_J_obj_;         // object Jacobian in body frame
    KDL::Jacobian s_J_dot_obj_;     // object Jacobian dot in spatial frame
    KDL::Jacobian b_J_dot_obj_;     // object Jacobian dot in body frame
    KDL::Twist s_V_obj_;            // object twist in spatial frame
    KDL::Twist b_V_obj_;            // object twist in body frame
    bool obj_added;

   //contacts
    std::vector<Contact> contacts_;
    std::vector<KDL::Frame> ee_F_c; // contact frames in end-effector frame
    std::vector<Eigen::Matrix<double,3,24>> b_J_c_; // contact Jacobian in body frame

     
 };



#endif
   


