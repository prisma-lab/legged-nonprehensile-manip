#ifndef QUADRUPEDCONTROL
#define QUADRUPEDCONTROL

#include <cstdlib>
#include <iostream>
// Eigen headers 
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include"dogbot_model/quadruped.h"

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

class QUADRUPEDController
{

 public:

     QUADRUPEDController();
   
     QUADRUPEDController(QUADRUPED &quadruped_);
     
     
   Eigen::VectorXd Cntr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::MatrixXd vdotswdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot
                            );
      Eigen::VectorXd CntrBr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::MatrixXd vdotswdes,
                            QUADRUPED::SWING_LEGS swinglegs,
                            Eigen::MatrixXd vdotobjdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot
                            );

      Eigen::VectorXd CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg,
                            Eigen::MatrixXd vdotobjdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot
                            );     



 private:

     QUADRUPED*  dogbot;

};

#endif
