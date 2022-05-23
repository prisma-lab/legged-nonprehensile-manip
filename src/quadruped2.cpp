#include "dogbot_model/quadruped.h"
#include "dogbot_model/alglib/optimization.h"
#include "Td_matrix.cpp"

QUADRUPED::QUADRUPED()
{
};

QUADRUPED::QUADRUPED(std::string modelFile)
{   //std::string modelFile="/home/viviana/catkin_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";
	
	// Create the robot 
	createrobot(modelFile);
    model = kinDynComp.model();
	
	// Resize matrices of the class given the number of DOFs
    n=model.getNrOfDOFs();
    robot_mass=model.getTotalMass();
    jointPos=iDynTree::VectorDynSize(n);
    baseVel=iDynTree::Twist();
    jointVel=iDynTree::VectorDynSize(n);
	q=iDynTree::VectorDynSize(6+n);
	dq=iDynTree::VectorDynSize(6+n);
	qb=iDynTree::VectorDynSize(6+n);
	dqb=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jobj=iDynTree::MatrixDynSize(6,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	Jac_lin=iDynTree::MatrixDynSize(12,6+n);
	JacDot=iDynTree::MatrixDynSize(24,6+n);
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
    MassMatrixCOM=iDynTree::FreeFloatingMassMatrix(model) ;
    BiasCOM=iDynTree::VectorDynSize(n+6);
	GravMatrixCOM=iDynTree::MatrixDynSize(n+6,1);
	JacCOM=iDynTree::MatrixDynSize(24,6+n);
	JacCOM_lin=iDynTree::MatrixDynSize(12,6+n);
	JdqdCOM=iDynTree::MatrixDynSize(24,1);
	JdqdCOM_lin=iDynTree::MatrixDynSize(12,1);
	LambdaObj=iDynTree::MatrixDynSize(6,6);
	BiasObj=iDynTree::VectorDynSize(6);
	Jobjdot=iDynTree::MatrixDynSize(6,6+n);
	std::string desc=kinDynComp.getDescriptionOfDegreesOfFreedom();
	std::cout<<"descrizione"<<desc<<std::endl;

	for( int i=0; i<25; i++)
	{
		std::cout<<"frame "<<kinDynComp.getFrameName(i)<<std::endl;

	}
}



//Update elements of the class given the new state

void QUADRUPED::update (Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,18,1> &eigenJointPos, Eigen::Matrix<double,18,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity)
{   

   // Update joints, base and gravity from inputs
	
	 iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
     iDynTree::toEigen(jointPos) = eigenJointPos;
     iDynTree::fromEigen(baseVel,eigenBasevel);
     toEigen(jointVel) = eigenJointVel;
     toEigen(gravity)  = eigenGravity;

	//Set the state for the robot 
	 kinDynComp.setRobotState(world_H_base,jointPos,
                              baseVel,jointVel,gravity);



    // Compute Center of Mass 
	iDynTree::Position xobj_lin= kinDynComp.getWorldTransform(24).getPosition();

	

    iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();

    toEigen(xobj)<<toEigen(xobj_lin),
	               toEigen(base_angle);
     
	 toEigen(CoM)<<toEigen(kinDynComp.getCenterOfMassPosition()),
	               toEigen(base_angle);
		   
	//Compute velocity of the center of mass

    Eigen::Matrix<double,3,1> CoM_vel_lin=toEigen(kinDynComp.getCenterOfMassVelocity());
	Eigen::Matrix<double,3,1> CoM_vel_ang=eigenBasevel.block(3,0,3,1);

   
	iDynTree::Twist xobjvel_=kinDynComp.getFrameVel(24);

	toEigen(xobjvel)=toEigen(xobjvel_);

	toEigen(CoM_vel)<<CoM_vel_lin,
	                  CoM_vel_ang;
		   
    // Compute position base +joints
	toEigen(qb)<<toEigen(world_H_base.getPosition()),
	            toEigen(base_angle),
	            eigenJointPos;

    std::cout<<"qb "<<toEigen(qb)<<std::endl;
    // Compute position COM+joints
	toEigen(q)<<toEigen(CoM),
	            eigenJointPos;

    std::cout<<"q "<<toEigen(q)<<std::endl;
	// Compute velocity COM+joints

	toEigen(dq)<<toEigen(CoM_vel),
	             eigenJointVel;

    std::cout<<"dq "<<toEigen(dq)<<std::endl;
	// Compute velocity base+joints

	toEigen(dqb)<<eigenBasevel,
	             eigenJointVel;

    std::cout<<"dqb "<<toEigen(dqb)<<std::endl;
	// Joint limits

    toEigen(qmin)<<-1.256636, -1.75 , -1.75,-1.75,-1.75,-1.58, -2.62, -3.15, -0.02, -1.58, -2.62,   -3.15, -0.02, -0.74, -1.598, 0, -2.08,-0.52;
    toEigen(qmax)<<1.3, 1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62, 3.15, 0.02,  1.58, 2.62,  0.83, 0.068, 2.21, 1.570795, 1.570795;

  
	// Get mass, bias (C(q,v)*v+g(q)) and gravity (g(q)) matrices
        //Initialize ausiliary vector
	     iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
	     iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
	
	  //Compute Mass Matrix
	
	     kinDynComp.getFreeFloatingMassMatrix(MassMatrix); 
	    
	  //Compute Coriolis + gravitational terms (Bias)
	
	     kinDynComp.generalizedBiasForces(bias_force);
	    
	     toEigen(Bias)<<iDynTree::toEigen(bias_force.baseWrench()),
	                   iDynTree::toEigen(bias_force.jointTorques());
         std::cout<<"bias"<<toEigen(Bias)<<std::endl;
	  //Compute Gravitational term
	
	     kinDynComp.generalizedGravityForces(grav_force);

	     toEigen(GravMatrix)<<iDynTree::toEigen(grav_force.baseWrench()),
	                        iDynTree::toEigen(grav_force.jointTorques());
     
	  //Compute Jacobian term

	     computeJac();	

	  // Compute Bias Acceleration -> J_dot*q_dot
	     computeJacDotQDot();

	  // Velocity vector (base+joints)
	     Eigen::Matrix<double, 24,1> q_dot;
	               
         q_dot<< eigenBasevel,
	             eigenJointVel;

	  // Compute Jacobian derivative
	     computeJacDot(q_dot);


	  // Compute Matrix needed for transformation from floating base representation to CoM representation

	     computeTransformation(q_dot);
		 std::cout<<"Trans"<<toEigen(T).inverse().transpose()<<std::endl;
		 std::cout<<"TransInv"<<toEigen(T_inv_dot)<<std::endl;

	  // Compute Mass Matrix in CoM representation 
	  
	  kinDynComp.getFrameFreeFloatingJacobian(24,Jobj);
	     
		 
      toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();


       Eigen::Matrix<double,6,6> M=toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Jobj).transpose();
	   toEigen(LambdaObj)=M.inverse();
	  
	   toEigen(BiasObj)=toEigen(LambdaObj)*toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Bias)-toEigen(LambdaObj)*toEigen(Jobjdot)*toEigen(dq);

       //Eigen::Matrix<double,6,1> g=toEigen(Jobj).completeOrthogonalDecomposition().pseudoInverse()*toEigen(Bias);
     	//   std::cout<<"massmatrix2"<<toEigen(Jobj).completeOrthogonalDecomposition().pseudoInverse()<<std::endl;
		//std::cout<<"massmatrix2"<<g<<std::endl;
	  // Compute Coriolis+gravitational term in CoM representation
	
	 toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);
         std::cout<<"biascom"<<toEigen(BiasCOM)<<std::endl;
	  // Compute gravitational term in CoM representation
	
	     toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
	
	  // Compute Jacobian term in CoM representation
	  
	     toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
	     std::cout<<"jaccom"<<toEigen(JacCOM)<<std::endl;
	     ComputeJaclinear();

		 ComputeJacstlinear();

	  // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
	
	     toEigen(JdqdCOM)=toEigen(JacDot)*toEigen(dq)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
	  
	     computeJdqdCOMlinear();

		 // Frame back right leg

         iDynTree::Transform  World_br;
		 World_br=kinDynComp.getWorldTransform(8);

		std::cout<<"pos br"<<toEigen(World_br.getPosition())<<std::endl;
		std::cout<<"rot br"<<toEigen(World_br.getRotation())<<std::endl;

        iDynTree::Twist br_vel;
		br_vel=kinDynComp.getFrameVel(8);

		std::cout<<"vel lin br"<<toEigen(br_vel.getLinearVec3() )<<std::endl;
		std::cout<<"vel ang br"<<toEigen(br_vel.getAngularVec3())<<std::endl;


        



	

  
	
}

// create robot
void QUADRUPED::createrobot(std::string modelFile)
{  bool ok = mdlLoader.loadModelFromFile(modelFile);

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
   
    // Create a KinDynComputations class from the model
    
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }
    
  

}


// Compute Jacobian
void  QUADRUPED::computeJac()
{    
    //Set ausiliary matrices
	  iDynTree::MatrixDynSize Jac1(6,6+n);
	  iDynTree::MatrixDynSize Jac2(6,6+n);
	  iDynTree::MatrixDynSize Jac3(6,6+n);
	  iDynTree::MatrixDynSize Jac4(6,6+n);
	
	// Compute Jacobian for each leg
	
	   // Jacobian for back right leg
       kinDynComp.getFrameFreeFloatingJacobian(13,Jac1);

	   // Jacobian for back left leg
	   kinDynComp.getFrameFreeFloatingJacobian(16,Jac2);

	  // Jacobian for front left leg
	  kinDynComp.getFrameFreeFloatingJacobian(10,Jac3);

	  // Jacobian for front right leg
	  kinDynComp.getFrameFreeFloatingJacobian(19,Jac4);

	 // Full Jacobian
	 toEigen(Jac)<<toEigen(Jac1),
	               toEigen(Jac2),
	               toEigen(Jac3),
	               toEigen(Jac4);
	
}

void QUADRUPED::ComputeJaclinear()
{
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(JacCOM_lin)=B*toEigen(JacCOM);

}

void QUADRUPED::ComputeJacstlinear()
{
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(Jac_lin)=B*toEigen(Jac);

}

// Compute Bias acceleration: J_dot*q_dot
void  QUADRUPED::computeJacDotQDot()
	
{
	   
	  // Bias acceleration for back right leg
	  iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(13); 
	  std::cout<<"frame 13 "<<kinDynComp.getFrameName(13)<<std::endl;
	  
	  // Bias acceleration for back left leg
	  iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc(16); 
	  std::cout<<"frame 16 "<<kinDynComp.getFrameName(16)<<std::endl;

	  // Bias acceleration for front left leg
	  iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc(10); 
	  std::cout<<"frame 10 "<<kinDynComp.getFrameName(10)<<std::endl;

	  // Bias acceleration for front right leg
	  iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc(19); 
	  std::cout<<"frame  19"<<kinDynComp.getFrameName(19)<<std::endl;
	
	  // Compute Bias acceleration for all the legs (backright-backleft-frontleft-frontright)
	
      toEigen(Jdqd)<<toEigen(Jdqd1),
                     toEigen(Jdqd2),
	                 toEigen(Jdqd3),
                     toEigen(Jdqd4);
	
	
}



//Method to get directly Jacobian time derivative, slower

void  QUADRUPED::computeJacDot(Eigen::Matrix<double,24,1> Vel_)
{    
	
 
	 // set ausiliary matrices
	iDynTree::MatrixDynSize Jac;
	Jac=iDynTree::MatrixDynSize(6,6+n);
    iDynTree::Twist vel_foot;

	Eigen::MatrixXd Jac_;
	Jac_=Eigen::MatrixXd::Zero(6,6+n);
	
	Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd jac_=Eigen::MatrixXd::Zero(6,6+n);
	Eigen::MatrixXd Jacdot=Eigen::MatrixXd::Zero(24,6+n);
	
	// Compute derivative for each leg
	int k=0;
	for (unsigned int k=0; k<4; ++k)
	
	{   
	// Getting Jacobian of the leg
		kinDynComp.getFrameFreeFloatingJacobian(8+3*k,Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(8+3*k);

    for(unsigned int i=0;i<6+n;++i)
    {  
     // Getting column i derivative
		
            for(unsigned int j=0;j<6+n;++j)
            {
                // Column J is the sum of all partial derivatives  ref (41)
                    jac_dot_k_ += getPartialDerivativeJac(Jac_,j,i)*Vel_(j) ;
				//*
            }
		
        jac_.col(i)= jac_dot_k_ ;
        jac_dot_k_=Eigen::VectorXd::Zero(6);
		
        }
	 	
	 Jac_=Eigen::MatrixXd::Zero(6,18);
	 Jacdot.block(0+6*k,6,6,18)<<jac_.block(0,6,6,18);

    Eigen::Matrix<double,6,1> xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	Eigen::Matrix<double,3,3> xic_hat_dot;
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(0+6*k,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);
     
	
	

} 

	toEigen(JacDot)=Jacdot;
	
}


// Compute partial time derivative
Eigen::VectorXd QUADRUPED::getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx)
{   // column's indices
    int j=joint_idx;
    int i=column_idx;
	
	// get columns
	Eigen::VectorXd jac_j_ = Jacobian.col(j);
    Eigen::VectorXd jac_i_ = Jacobian.col(i);
	
	// Get linear and rotational parts
	Eigen::Vector3d jac_j_vel = jac_j_.head(3);
    Eigen::Vector3d jac_i_vel = jac_i_.head(3);
	
	Eigen::Vector3d jac_j_rot = jac_j_.tail(3);
    Eigen::Vector3d jac_i_rot = jac_i_.tail(3);
	
	// Ausiliary vector
	Eigen::Vector3d t_djdq_vel=Eigen::Vector3d::Zero();
	Eigen::Vector3d t_djdq_rot=Eigen::Vector3d::Zero();
	
	if(j < i)
    {
        // P_{\Delta}({}_{bs}J^{j})  ref (20)
        t_djdq_vel = jac_j_rot.cross(jac_i_vel);
        t_djdq_rot = jac_j_rot.cross(jac_i_rot);
		
    }else if(j > i)
    {
        // M_{\Delta}({}_{bs}J^{j})  ref (23)
        
        t_djdq_vel = -jac_j_vel.cross(jac_i_rot);
		
    }else if(j == i)
    {
         // ref (40)
         
         t_djdq_vel = jac_i_rot.cross(jac_i_vel);
		
    }
	
	
	Eigen::VectorXd t_djdq_=Eigen::VectorXd::Zero(6);
	t_djdq_<<t_djdq_vel,
	         t_djdq_rot;
	
    return t_djdq_;
	
	

}


void QUADRUPED::computeJdqdCOMlinear()
{
	Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);


    toEigen(JdqdCOM_lin)= Eigen::MatrixXd::Zero(12,1);
    toEigen(JdqdCOM_lin)=B*toEigen(JdqdCOM);
	
}


// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void QUADRUPED::computeTransformation(Eigen::VectorXd Vel_)
{   
	//Set ausiliary matrices
	iDynTree::MatrixDynSize Jb(6,6+n);
	iDynTree::MatrixDynSize Jbc(6,n);
	iDynTree::Vector3 xbc;
	iDynTree::MatrixDynSize xbc_hat(3,3);
	iDynTree::MatrixDynSize xbc_hat_dot(3,3);
	iDynTree::MatrixDynSize Jbc_dot(6,6+n);
	iDynTree::Vector3 xbo_dot;
	
	// Compute T matrix
	    // Get jacobians of the floating base and of the objectframe
	    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
        kinDynComp.getCenterOfMassJacobian(Jcom);

	    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
	    	    toEigen(Jbc)<<toEigen(Jcom).block<3,18>(0,6)-toEigen(Jb).block<3,18>(0,6);
	
	    // Get xb (floating base position) and xc ( com position)
	    iDynTree::Position xb = world_H_base.getPosition();
        iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();
		iDynTree::Transform  Tobj;
		Tobj=kinDynComp.getWorldTransform(24);
	    iDynTree::Position xobj= Tobj.getPosition();
		
	
	    // Vector xcb=xc-xb
	    toEigen(xbc)=toEigen(xc)-toEigen(xb);
	
	    // Skew of xcb
	    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
	                      toEigen(xbc)[2], 0, -toEigen(xbc)[0],                          
	                     -toEigen(xbc)[1], toEigen(xbc)[0], 0;
	
	
     	// Matrix T for the transformation
	    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), -toEigen(xbc_hat), toEigen(Jbc),
	                Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,18),
	                Eigen::MatrixXd::Zero(18,3),  Eigen::MatrixXd::Zero(18,3), Eigen::MatrixXd::Identity(18,18);
	
	
	
	//Compute time derivative of T 
      // Compute derivative of xbc
	 /*  toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
	
	   //Compute skew of xbc
   	   toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
	                      toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
	                      -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

	
	   // Time derivative of Jbc
	   kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);
	  
	  Eigen::Matrix<double,6,12> jac_;
	  Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);

	  TDMAT Tj(toEigen(qb),toEigen(dqb));
	  Eigen::Matrix<double,3,12> Jbcdot=Tj.getTdmatrix();

	   // Tdot matrix
	   toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), Jbcdot,
                      Eigen::MatrixXd::Zero(15,18);*/
	 
	
	  
	 /* Eigen::Matrix<double,6,24> jac_;
	  Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);
	
	   for(unsigned int i=0;i<6+n;++i)
         {  
            // Getting column i derivative
		
            for(unsigned int j=0;j<6+n;++j)
            {
                // Column J is the sum of all partial derivatives  ref (41)
                    jac_dot_k_ += getPartialDerivativeJac(toEigen(Jobj),j,i)*Vel_(j) ;
				
            }
		
        jac_.col(i)= jac_dot_k_ ;
        jac_dot_k_=Eigen::VectorXd::Zero(6);
		
        }
	   
	    toEigen(Jobjdot)<<jac_;
	 

	  
	

	   // Tdot matrix
	   toEigen(T_inv_dot)<<toEigen(Jobjdot),
                      Eigen::MatrixXd::Zero(18,24);*/
	
	
}


// return Mass Matrix
Eigen::MatrixXd QUADRUPED::getMassMatrix()
{
	
	return toEigen(MassMatrix);

}


//return Bias Matrix
Eigen::VectorXd QUADRUPED::getBiasMatrix()
{

	return toEigen(Bias);

}

//return gravity term
Eigen::VectorXd QUADRUPED::getGravityMatrix()
{
	
	return toEigen(GravMatrix);

}


//return Jacobian
Eigen::MatrixXd QUADRUPED::getJacobian()
{
	
	return toEigen(Jac);

}


//return Bias Acceleration --> J_dot*q_dot
Eigen::MatrixXd QUADRUPED::getBiasAcc()
{

	return toEigen(Jdqd);

}

//return matrix T
Eigen::MatrixXd QUADRUPED::getTransMatrix()
{

	return toEigen(T);

}

// return Mass Matrix in COM representation
Eigen::MatrixXd QUADRUPED::getMassMatrixCOM()
{
	
	return toEigen(MassMatrixCOM);

}


//return Bias Matrix in COM representation
Eigen::VectorXd QUADRUPED::getBiasMatrixCOM()
{
	
	return toEigen(BiasCOM);

}

//return gravity term in COM representation
Eigen::VectorXd QUADRUPED::getGravityMatrixCOM()
{

	return toEigen(GravMatrixCOM);

}


//return Jacobian in COM representation
Eigen::MatrixXd QUADRUPED::getJacobianCOM()
{

	return toEigen(JacCOM);

}

Eigen::MatrixXd QUADRUPED::getJacobianCOM_linear()
{
    

	return toEigen(JacCOM_lin);

}


//return Bias Acceleration --> J_dot*q_dot in COM representation
Eigen::MatrixXd QUADRUPED::getBiasAccCOM()
{
	
	return toEigen(JdqdCOM);

}

// Return Bias accelration in CoM representation
Eigen::MatrixXd QUADRUPED::getBiasAccCOM_linear()
{
	return toEigen(JdqdCOM_lin);

}


Eigen::MatrixXd QUADRUPED::getCOMpos()
{

	
 return toEigen(CoM);
}
  
Eigen::MatrixXd QUADRUPED::getOBJpos()
{

	
 return toEigen(xobj);
}
    
Eigen::MatrixXd QUADRUPED::getCOMvel()
{

 return toEigen(CoM_vel);

}

Eigen::MatrixXd QUADRUPED::getOBJvel()
{

 return toEigen(xobjvel);

}

double QUADRUPED::getMass()
{
	return robot_mass;
}


int QUADRUPED::getDoFsnumber()
{
	return n;
}


Eigen::MatrixXd QUADRUPED::getMassMatrixCOM_com()
{

	return toEigen(MassMatrixCOM).block(0,0,6,6);
   
}


Eigen::MatrixXd QUADRUPED::getMassMatrixCOM_joints()
{
   
	return toEigen(MassMatrixCOM).block(6,6,n,n);
}

 Eigen::MatrixXd QUADRUPED::getJobj()
 {
     return toEigen(Jobj);
 };
  Eigen::MatrixXd QUADRUPED::getLambdaObj()
  {
     return toEigen(LambdaObj);
  };
  Eigen::MatrixXd QUADRUPED::getBiasObj()
  {
      return toEigen(BiasObj);
  };
// Quadratic problem
Eigen::MatrixXd QUADRUPED::qpproblem( Eigen::Matrix<double,6,1> &Wcom_des)
{

	// Set variables
   int variables=36;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(110,37);
   Lt.setlength(110);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
   std::cout<<"Jcomcom"<<Jstcom<<std::endl;

   Eigen::Matrix<double, 12, 18> Jstj= toEigen(JacCOM_lin).block(0,6,12,18);

   // cost function quadratic matrix
   Eigen::Matrix<double,12,36> Sigma= Eigen::Matrix<double,12,36>::Zero();
   Sigma.block(0,24,12,12)= Eigen::Matrix<double,12,12>::Identity();
   Eigen::Matrix<double, 6, 12> Jot=toEigen(LambdaObj)*toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Jac_lin).transpose();
   Eigen::Matrix<double,6,36>  T_s= Jot*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,36,36> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,36,36> eigenQ= eigenQ2+Eigen::Matrix<double,36,36>::Identity();
 
   
	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,36,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(36,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,18, 36> eigenA= Eigen::Matrix<double,18,36>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,24,6,12)=-Jstcom.transpose();

    eigenA.block(6,0,12,6)=Jstcom;

    eigenA.block(6,6,12,18)=Jstj;

    // Known term
    Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);;

	eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);


    
    

	//Inequality Constraints

	Eigen::Matrix<double,92, 36> eigenD= Eigen::Matrix<double,92,36>::Zero();
	
	 // Torque limits
	eigenD.block(20,6,18,18)=toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(20,24,18,12)=-Jstj.transpose();

	eigenD.block(38,6,18,18)=-toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(38,24,18,12)=Jstj.transpose();

	eigenD.block(56,6,18,18)=Eigen::Matrix<double,18,18>::Identity();

    eigenD.block(74,6,18,18)=-Eigen::Matrix<double,18,18>::Identity();
    
	//Friction
	   double mu=1000;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,24,20,12)=Dfr;


    // Known terms for inequality
	Eigen::Matrix<double,92, 1> eigenC= Eigen::Matrix<double,92,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,18,1> tau_max=60*Eigen::Matrix<double,18,1>::Ones();
	Eigen::Matrix<double,18,1> tau_min=-60*Eigen::Matrix<double,18,1>::Ones();

    Eigen::Matrix<double,18, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,18,1);

	eigenC.block(20,0,18,1)=tau_max-eigenBiascom;
	eigenC.block(38,0,18,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,18, 1> eigenq=toEigen(q).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigendq=toEigen(dq).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,18, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,18, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,18, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(56,0,18,1)=ddqmax;
	 eigenC.block(74,0,18,1)=-ddqmin;
	
	
     //Linear constraints matrix
    Eigen::Matrix<double,110, 37> eigenL= Eigen::Matrix<double,110,37>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 18)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
    


    //Bounded constraints
     /*Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;*/


    
    // Set qp
   // alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	//alglib::minqpsetalgodenseipm(state, 1.0e-9);
     alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);

    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

    Eigen::VectorXd x_eigen= Eigen::VectorXd::Zero(36) ;
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);

     std::cout<<"solution "<<x_eigen<<std::endl;

     Eigen::Matrix<double,6,1> Wobt=Jot*x_eigen.block(24,0,12,1);

	 std::cout<<"Wobt "<<Wobt<<std::endl;
    
	Eigen::Matrix<double,18,1> tau= Eigen::Matrix<double,18,1>::Zero();
	tau=toEigen(MassMatrixCOM).block(6,6,18,18)*x_eigen.block(6,0,18,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(24,0,12,1);
	return tau;

}

// Quadratic problem back right leg
Eigen::VectorXd QUADRUPED::qpproblembr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes)
{

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(84,31);
   Lt.setlength(84);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 9, 6> Jstcom= toEigen(JacCOM_lin).block(3,0,9,6);
   Eigen::Matrix<double, 9, 12> Jstj= toEigen(JacCOM_lin).block(3,6,9,12);

   Eigen::Matrix<double, 3, 6> Jswcom= toEigen(JacCOM_lin).block(0,0,3,6);
   Eigen::Matrix<double, 3, 12> Jswj= toEigen(JacCOM_lin).block(0,6,3,12);
   // cost function quadratic matrix
   Eigen::Matrix<double,9,30> Sigma= Eigen::Matrix<double,9,30>::Zero();
   Sigma.block(0,18,9,9)= Eigen::Matrix<double,9,9>::Identity();
   

   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
   eigenR.block(27,27,3,3)=10000000*Eigen::Matrix<double,3,3>::Identity();
   
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,15, 30> eigenA= Eigen::Matrix<double,15,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,9)=-Jstcom.transpose();

    eigenA.block(6,0,9,6)=Jstcom;

    eigenA.block(6,6,9,12)=Jstj;

	//std::cout<<"eigenA"<<eigenA<<std::endl;

    // Known term
    Eigen::Matrix<double,15, 1> eigenb= Eigen::Matrix<double,15,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);;

	eigenb.block(6,0,9,1)=-toEigen(JdqdCOM_lin).block(3,0,9,1);

    //std::cout<<"eigenB"<<eigenb<<std::endl;
    
    

	//Inequality Constraints

	Eigen::Matrix<double,69, 30> eigenD= Eigen::Matrix<double,69,30>::Zero();
	
	 // Torque limits
	eigenD.block(15,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(15,18,12,9)=-Jstj.transpose();

	eigenD.block(27,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(27,18,12,9)=Jstj.transpose();
    
    eigenD.block(39,0,3,6)=Jswcom;

    eigenD.block(39,6,3,12)=Jswj;

	eigenD.block(39,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(42,0,3,6)=-Jswcom;

    eigenD.block(42,6,3,12)=-Jswj;

    eigenD.block(42,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(45,6,12,12)=Eigen::Matrix<double,12,12>::Identity();

    eigenD.block(57,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
    
	//Friction
	   double mu=1;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,15,9> Dfr=Eigen::Matrix<double,15,9>::Zero();

		for(int i=0; i<3; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,18,15,9)=Dfr;

    //std::cout<<"eigenD"<<eigenD<<std::endl;
    // Known terms for inequality
	Eigen::Matrix<double,69, 1> eigenC= Eigen::Matrix<double,69,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);

	eigenC.block(15,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(27,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(45,0,12,1)=ddqmax;
	 eigenC.block(57,0,12,1)=-ddqmin;
	
	 eigenC.block(39,0,3,1)= vdotswdes;
	 eigenC.block(42,0,3,1)= -vdotswdes;
	
	  // std::cout<<"eigenc"<<eigenC<<std::endl;
     //Linear constraints matrix
    Eigen::Matrix<double,84, 31> eigenL= Eigen::Matrix<double,84,31>::Zero();

   
	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 15)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
    }
    


    //Bounded constraints
     /*Eigen::Matrix<double,30, 1> eigenBL= Eigen::Matrix<double,30,1>::Zero();
     
	 eigenBL.block(6,0,12,1)=ddqmin;

     Eigen::Matrix<double,30, 1> eigenBU= Eigen::Matrix<double,30,1>::Zero();
	
	 eigenBU.block(6,0,12,1)=ddqmax;

     for ( int i = 0; i < 30; i++ )
    {  
		if (i<6)
	  {
       bl(i) = alglib::fp_neginf;
       bu(i) = alglib::fp_posinf;
	   }
       else if (i>=6 && i<18)
	   { bl(i) = eigenBL(i,0);
	     bu(i) = eigenBU(i,0);
	    }
		else
		{
	    bl(i) = alglib::fp_neginf;
        bu(i) = alglib::fp_posinf;

		}
    };

    bl(20)=0;
    bl(23)=0;
    bl(26)=0;
	bl(29)=0;*/


    
    // Set qp
   // alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);


    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

    Eigen::VectorXd x_eigen= Eigen::VectorXd::Zero(30) ;
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);




    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(18,0,9,1);
	return tau;

}
