#include "dogbot_model/quadruped_control.h"


QUADRUPEDController::QUADRUPEDController()
{

}

QUADRUPEDController::QUADRUPEDController(QUADRUPED &quadruped_)

{
    dogbot = &quadruped_;
  
}


// Control 
Eigen::VectorXd QUADRUPEDController::Cntr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::MatrixXd vdotswdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot
                            )
{

// Compute deltax, deltav

 Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();

std::cout<<"dx"<<deltax<<std::endl;
std::cout<<"dv"<<deltav<<std::endl;

// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
   //Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  //Eigen::MatrixXd M_com=dogbot->getLambdaObj();
   Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
  Eigen::MatrixXd M=dogbot->getMassMatrix();

  Eigen::MatrixXd J_obj=dogbot->getJobj();

  Eigen::Matrix<double,24,1> tau=Eigen::Matrix<double,24,1>::Zero();
  //std::cout<<"tau"<<tauj<<std::endl;
  //tau.block(6,0,18,1)=tauj;
  
  Eigen::Matrix<double,6,1> tauobj=M_com*J_obj*M.inverse()*tau;

  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);
    std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Control vector
  Eigen::VectorXd tauopt= Eigen::VectorXd::Zero(18);

Eigen::MatrixXd Rot=dogbot->getRotwrench();

 std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Solve quadratic problem
  tauopt=dogbot->qpproblem(Wcom_des, vdotswdes, _desObjAcc, e, edot);
  std::cout<<"prova0"<<std::endl;
  return tauopt;


}


Eigen::VectorXd QUADRUPEDController::CntrBr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::MatrixXd vdotswdes,
                            QUADRUPED::SWING_LEGS swinglegs,
                            Eigen::MatrixXd vdotobjdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot
                            )
{

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,24,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();

  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);
  
// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
 std::cout<<"deltax"<<deltax<<std::endl;
  std::cout<<"deltay"<<deltav<<std::endl;
    std::cout<<"M_com"<<M_com<<std::endl;
      std::cout<<"CoMAccDes"<<toEigen(CoMAccDes)<<std::endl;
    std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Solve quadratic problem
  tau=dogbot->qpproblembr(Wcom_des, vdotswdes, swinglegs, vdotobjdes, _desObjAcc, e, edot);
  
  return tau;


}

Eigen::VectorXd QUADRUPEDController::CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg,
                            Eigen::MatrixXd vdotobjdes, 
                            Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot)
{
  int swl1, stl1, stl2, stl3;
    switch(swingleg){
		case QUADRUPED::BR: swl1=0; 
		stl1=3;
		stl2=6 ; 
		stl3=9;
		break;
		case QUADRUPED::FL: swl1=6;
		stl1=0;
		stl2=3 ; 
		stl3=9;
		break;
		case QUADRUPED::BL: swl1=3;
		stl1=0;
		stl2=6; 
		stl3=9;
		break;
		case QUADRUPED::FR: swl1=9;
		stl1=0;
		stl2=3; 
		stl3=6;
		 break;
	}

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,24,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
  
  Eigen::Matrix<double,12,24> J=dogbot->getJacobianCOM_linear();
  Eigen::Matrix<double,9,6> Jcom;
   Jcom.block(0,0,3,6)=J.block(stl1,0,3,6);
   Jcom.block(3,0,3,6)=J.block(stl2,0,3,6);
   Jcom.block(6,0,3,6)=J.block(stl3,0,3,6);


  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);
 std::cout<<"Wcomdes"<<Wcom_des<<std::endl;
// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(18);

// Solve quadratic problem
  tau=dogbot->qpproblemol(Wcom_des, vdotswdes, swingleg, vdotobjdes, _desObjAcc, e, edot);
  
  return tau;


}
