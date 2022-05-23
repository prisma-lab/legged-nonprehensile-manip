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
	JacDotObj=iDynTree::MatrixDynSize(6,6+n);
	Rotwrench=iDynTree::MatrixDynSize(6,6);
	obj_added=false;
	x_eigen= Eigen::VectorXd::Zero(52);
	b_J_c_.resize(4);
	std::string desc=kinDynComp.getDescriptionOfDegreesOfFreedom();
	std::cout<<"descrizione"<<desc<<std::endl;
	    for( int i=0; i<kinDynComp.getNrOfFrames(); i++ )
	{
		std::cout<<"frame"<<i<<" : "<<kinDynComp.getFrameName(i)<<"\n"<<std::endl;
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
	iDynTree::Position xobj_lin= kinDynComp.getWorldTransform(kinDynComp.getFrameIndex("wrist_flexion_link")).getPosition();

	

    iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();
    iDynTree::Vector3 obj_angle=kinDynComp.getWorldTransform(kinDynComp.getFrameIndex("wrist_flexion_link")).getRotation().asRPY();

    toEigen(xobj)<<toEigen(xobj_lin),
	               toEigen(obj_angle);
     std::cout<<"obj"<<toEigen(xobj)<<std::endl;

	 toEigen(CoM)<<toEigen(kinDynComp.getCenterOfMassPosition()),
	               toEigen(base_angle);

	std::cout<<"com"<<toEigen(CoM)<<std::endl;	   
	//Compute velocity of the center of mass

    Eigen::Matrix<double,3,1> CoM_vel_lin=toEigen(kinDynComp.getCenterOfMassVelocity());
	Eigen::Matrix<double,3,1> CoM_vel_ang=eigenBasevel.block(3,0,3,1);

   
	iDynTree::Twist xobjvel_=kinDynComp.getFrameVel(kinDynComp.getFrameIndex("wrist_flexion_link"));

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
    toEigen(qmax)<<1.3, 1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62, 3.15, 0.02,  1.58, 2.62,  0.83, 1.598, 2.21, 1.570795, 1.570795;

  
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
         computeJacDotObj(q_dot);

	  // Compute Matrix needed for transformation from floating base representation to CoM representation

	     computeTransformation(q_dot);
	

	  // Compute Mass Matrix in CoM representation 
	  
	  kinDynComp.getFrameFreeFloatingJacobian(kinDynComp.getFrameIndex("wrist_flexion_link"),Jobj);
	     
		 
      toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();

	  std::cout<<"MassMatrixCOM"<<toEigen(MassMatrixCOM)<<std::endl;

       Eigen::Matrix<double,6,6> M=toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Jobj).transpose();
	   toEigen(LambdaObj)=M.inverse();
	  
	   toEigen(BiasObj)=toEigen(LambdaObj)*toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Bias)-toEigen(LambdaObj)*toEigen(Jobjdot)*toEigen(dqb);
       std::cout<<"jobj"<<toEigen(Jobj)<<std::endl;



		

       //Eigen::Matrix<double,6,1> g=toEigen(Jobj).completeOrthogonalDecomposition().pseudoInverse()*toEigen(Bias);
     	//   std::cout<<"massmatrix2"<<toEigen(Jobj).completeOrthogonalDecomposition().pseudoInverse()<<std::endl;
		//std::cout<<"massmatrix2"<<g<<std::endl;
	  // Compute Coriolis+gravitational term in CoM representation
	
	 toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);
         std::cout<<"biascom"<<toEigen(BiasObj)<<std::endl;
	  // Compute gravitational term in CoM representation
	
	     toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
	
	  // Compute Jacobian term in CoM representation
	  
	     toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
		
         toEigen(JacDotObj)=toEigen(Jobjdot)*toEigen(T).inverse()+toEigen(Jobj)*toEigen(T_inv_dot);

		 //toEigen(Jobj)=toEigen(Jobj)*toEigen(T).inverse();
		
	
	     ComputeJaclinear();

		 ComputeJacstlinear();

	  // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
	
	     toEigen(JdqdCOM)=toEigen(Jdqd)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
	
	     computeJdqdCOMlinear();

		 // Frame back right leg

         iDynTree::Transform  World_br;
	

        //std::cout<<"ci"<<toEigen(Jac_lin).bdcSvd( Eigen::ComputeThinU |  Eigen::ComputeThinV).solve(Eigen::Matrix<double,12,12>::Identity())<<std::endl;
        

       Eigen::MatrixXd jinv= toEigen(Jac_lin).bdcSvd( Eigen::ComputeThinU |  Eigen::ComputeThinV).solve(Eigen::Matrix<double,12,12>::Identity());

        
		Eigen::MatrixXd P=Eigen::Matrix<double,24,24>::Identity()-jinv*toEigen(Jac_lin);
        
		Eigen::MatrixXd Mc=P*toEigen(MassMatrix)+Eigen::Matrix<double,24,24>::Identity()-P;
		Eigen::MatrixXd Mx= toEigen(Jobj)*Mc.inverse()*P*toEigen(Jobj).transpose();
		Eigen::MatrixXd Lambdac=Mx.inverse();

        Eigen::MatrixXd biasc= Lambdac*(toEigen(Jobj)*Mc.inverse()*(P*toEigen(Bias))-toEigen(Jobjdot)*toEigen(dqb));

        iDynTree::Vector6 xobjc;
		toEigen(xobjc)=toEigen(xobj)-toEigen(CoM);
		iDynTree::MatrixDynSize xobjc_hat(3,3);
         toEigen(xobjc_hat)<<0, -toEigen(xobjc)[2], toEigen(xobjc)[1],
	                      toEigen(xobjc)[2], 0, -toEigen(xobjc)[0],                          
	                     -toEigen(xobjc)[1], toEigen(xobjc)[0], 0;

      
 

        World_br=kinDynComp.getWorldTransform(24);

	     
		toEigen(Rotwrench)<< toEigen(World_br.getRotation()), Eigen::Matrix<double,3,3>::Zero(),
		                      toEigen(xobjc_hat)*toEigen(World_br.getRotation()), toEigen(World_br.getRotation()); 


		// Object part
		
         KDL::Twist s_T_f;
         KDL::Frame s_F_f;
         KDL::Jacobian s_J_f(24);
         KDL::Jacobian s_J_dot_f(24);
		 Eigen::Matrix<double,3,1> Bias_lin=toEigen(BiasObj).block(0,0,3,1);
         Eigen::Matrix<double,3,1> Bias_rot=toEigen(BiasObj).block(3,0,3,1);
		KDL::Vector  Linbias;
		Linbias=KDL::toKDLVector(Bias_lin);
		KDL::Vector  Angbias;
		Angbias=KDL::toKDLVector(Bias_rot);
         KDL::Twist s_J_dot_q_dot_f(Linbias,Angbias);
		
     

		iDynTree::Twist obj_vel;
	   	obj_vel=kinDynComp.getFrameVel(kinDynComp.getFrameIndex("wrist_flexion_link"));
        Eigen::Matrix<double,3,1> linvel_obj=toEigen(obj_vel).block(0,0,3,1);
        Eigen::Matrix<double,3,1> rotvel_obj=toEigen(obj_vel).block(3,0,3,1);
        KDL::Vector  Linvel;
		Linvel=KDL::toKDLVector(linvel_obj);
		KDL::Vector  Angvel;
		Angvel=KDL::toKDLVector(rotvel_obj);
        KDL::Twist twistobj(Linvel,Angvel);

        iDynTree::Transform  World_obj;
        World_obj=kinDynComp.getWorldTransform(kinDynComp.getFrameIndex("wrist_flexion_link"));
		std::cout<<"obj"<<toEigen(World_obj.getPosition())<<std::endl;

		Eigen::Matrix<double,3,3> rotation_obj=toEigen(World_obj.getRotation());
        Eigen::Matrix<double,3,1> position_obj=toEigen(World_obj.getPosition());
		KDL::Rotation Rotobj;
		Rotobj=KDL::toKDLRotation(rotation_obj); 
		KDL::Vector  Vecobj;
		Vecobj=KDL::toKDLVector(position_obj);
        KDL::Frame frameobj(Rotobj,Vecobj);

		KDL::FrameVel s_Fv_f(frameobj,twistobj);

		for (int i=0; i<24;i++)
		{
			Eigen::Matrix<double,6,1>   coli=toEigen(Jobj).block(0,i,6,1);
			Eigen::Matrix<double,6,1>   colidot=toEigen(Jobjdot).block(0,i,6,1);
			s_J_f.setColumn(i,KDL::toKDLTwist(coli));
			s_J_dot_f.setColumn(i,KDL::toKDLTwist(coli));
           
		}

		s_T_f = s_Fv_f.GetTwist();
        s_F_f = s_Fv_f.GetFrame();
		std::cout<<"s_F_f"<<s_J_f.data<<std::endl;
        std::cout<<"twist"<<linvel_obj<<std::endl;
		std::cout<<"twist"<<s_T_f<<std::endl;
   
    b_J_ee_ = KDL::Jacobian(24);
    s_J_dot_ee_ = KDL::Jacobian(24);
    b_J_dot_ee_ = KDL::Jacobian(24);
    b_J_obj_ = KDL::Jacobian(24);
    s_J_obj_ = KDL::Jacobian(24);
    s_J_dot_obj_= KDL::Jacobian(24);
    b_J_dot_obj_= KDL::Jacobian(24);
	s_J_ee_ = KDL::Jacobian(24);
    s_J_ee_.data.setZero();
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    b_J_obj_.data.setZero();
    s_J_obj_.data.setZero();
    s_J_dot_obj_.data.setZero();
    b_J_dot_obj_.data.setZero();
    // robot end-effector
	KDL::Vector  Vecee(0,0,0.056);
	KDL::Frame f_F_ee_ = KDL::Frame::Identity(); 
	f_F_ee_.p[2] = 0.086164;
    s_F_ee_ = s_F_f*f_F_ee_;
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);

    // object
	if (obj_added==true)
	{
	KDL::Frame obj_frame = s_F_ee_*ee_F_obj_;
	std::cout<<"s_F_f"<<obj_frame<<std::endl;
    obj_->setFrame(obj_frame);
    KDL::Vector s_p_ee_obj = obj_->getFrame().p - s_F_ee_.p;
    s_V_obj_ = s_V_ee_.RefPoint(s_p_ee_obj);
    b_V_obj_ = adjoint(obj_->getFrame().Inverse(), s_V_obj_);
    obj_->setBodyVelocity(b_V_obj_);
    obj_->setSpatialVelocity(s_V_obj_);
    KDL::changeRefPoint(s_J_ee_, s_p_ee_obj, s_J_obj_);
    KDL::changeRefPoint(s_J_dot_ee_, s_p_ee_obj, s_J_dot_obj_);
    KDL::changeBase(s_J_obj_, obj_->getFrame().M.Inverse(), b_J_obj_);
    KDL::changeBase(s_J_dot_obj_, obj_->getFrame().M.Inverse(), b_J_dot_obj_);
	
	
	//contacts
    for (unsigned int i = 0; i < contacts_.size(); i++)
    {
        // update contacts jacobian
        KDL::Jacobian J(24);
        KDL::changeRefFrame(s_J_ee_, s_F_ee_*contacts_[i].getFrame().Inverse(), J);
        // KDL::changeBase(J, obj_->getFrame().M.Inverse(), J);
        Eigen::Matrix<double,3,24> J_ci = obj_->getContacts().at(0).getB().transpose()*J.data;
        b_J_c_.at(i) = J_ci*toEigen(T).inverse();
		std::cout<<"contact"<<contacts_[i].getFrame().Inverse()<<std::endl;
		std::cout<<"contact"<<J_ci<<std::endl;
    }
	std::cout<<"contact B"<<obj_->getContacts().at(0).getB()<<std::endl;
	
}


        

}
Eigen::MatrixXd QUADRUPED::getRotwrench()
{return toEigen(Rotwrench);}

KDL::Frame QUADRUPED::getEEFrame()
{
    return s_F_ee_;
}


void QUADRUPED::setContacts(const std::vector<Contact> &_contact)
{
    contacts_.resize(_contact.size());
    for (unsigned int i = 0; i < _contact.size(); i++)
        contacts_.at(i) = _contact.at(i);
}

std::vector<Contact> QUADRUPED::getContacts() const
{
    return contacts_;
}

void QUADRUPED::addObj(KDLObject &_obj)
{
    obj_= &_obj;
    ee_F_obj_ = s_F_ee_.Inverse()*obj_->getFrame();

    std::cout << "s_T_obj: " << std::endl << obj_->getFrame() << std::endl;
    std::cout << "ee_T_obj: " << std::endl << ee_F_obj_ << std::endl;

    // compute contacts on the robot in the ee frame
    std::vector<Contact> robotContacts(obj_->getContacts().size());

    for (unsigned int i = 0; i < obj_->getContacts().size(); i++)
        robotContacts.at(i).setFrame(this->getEEFrame().Inverse()*obj_->getFrame()*obj_->getContacts()[i].getFrame());
    this->setContacts(robotContacts);

    std::cout << "added object with contact points: " << std::endl;
    for (unsigned int i = 0; i < this->getContacts().size(); i++)
        std::cout << "ee_T_c "<< i << std::endl << contacts_[i].getFrame() << std::endl;

    obj_added=true;
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
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_right_foot"), Jac1);

    // Jacobian for back left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_left_foot"),Jac2);

    // Jacobian for front left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_left_foot"), Jac3);

    // Jacobian for front right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_right_foot"), Jac4);


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
    iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc("back_right_foot"); 

    // Bias acceleration for back left leg
    iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc("back_left_foot"); 

    // Bias acceleration for front left leg
    iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc("front_left_foot"); 
    // Bias acceleration for front right leg
    iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc("front_right_foot"); 	
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
	  
	// Getting Jacobian of the leg
		kinDynComp.getFrameFreeFloatingJacobian(13,Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(13);

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
	 Jacdot.block(0,6,6,18)<<jac_.block(0,6,6,18);

    Eigen::Matrix<double,6,1> xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	Eigen::Matrix<double,3,3> xic_hat_dot;
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(0,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);

    // Back left leg
		kinDynComp.getFrameFreeFloatingJacobian(16,Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(16);

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
	 Jacdot.block(6,6,6,18)<<jac_.block(0,6,6,18);

     xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(6,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);

    // Front left leg
		kinDynComp.getFrameFreeFloatingJacobian(10,Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(10);

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
	 Jacdot.block(12,6,6,18)<<jac_.block(0,6,6,18);

     xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(12,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);

     // front right leg
	 	kinDynComp.getFrameFreeFloatingJacobian(19,Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(19);

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
	 Jacdot.block(18,6,6,18)<<jac_.block(0,6,6,18);

    xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(18,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);
     
	
	



	toEigen(JacDot)=Jacdot;
	
}

void  QUADRUPED::computeJacDotObj(Eigen::Matrix<double,24,1> Vel_)
{    
	
 
	 // set ausiliary matrices
	iDynTree::MatrixDynSize Jac;
	Jac=iDynTree::MatrixDynSize(6,6+n);
    iDynTree::Twist vel_foot;

	Eigen::MatrixXd Jac_;
	Jac_=Eigen::MatrixXd::Zero(6,6+n);
	
	Eigen::VectorXd jac_dot_k_=Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd jac_=Eigen::MatrixXd::Zero(6,6+n);
	Eigen::MatrixXd Jacdot=Eigen::MatrixXd::Zero(6,6+n);
	
	// Compute derivative for each leg
	int k=0;
	for (unsigned int k=0; k<4; ++k)
	
	{   
	// Getting Jacobian of the leg
	kinDynComp.getFrameFreeFloatingJacobian(kinDynComp.getFrameIndex("wrist_flexion_link"),Jac);
        Jac_= iDynTree::toEigen(Jac);
		vel_foot=kinDynComp.getFrameVel(kinDynComp.getFrameIndex("wrist_flexion_link"));

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
	 Jacdot.block(0,6,6,18)<<jac_.block(0,6,6,18);

    Eigen::Matrix<double,6,1> xic_dot=toEigen(vel_foot)-toEigen(baseVel);
    
	Eigen::Matrix<double,3,3> xic_hat_dot;
    xic_hat_dot<<0, -xic_dot[2], xic_dot[1],
	            xic_dot[2], 0, -xic_dot[0],                          
	            -xic_dot[1], xic_dot[0], 0;



     Jacdot.block(0,0,6,6)<<Eigen::MatrixXd::Zero(3,3), -xic_hat_dot,
	                            Eigen::MatrixXd::Zero(3,6) ;


	 jac_=Eigen::MatrixXd::Zero(6,6+n);
     
	
	

} 

	toEigen(Jobjdot)=Jacdot;
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
	iDynTree::MatrixDynSize Jbc(3,n);
	iDynTree::Vector3 xbc;
	iDynTree::MatrixDynSize xbc_hat(3,3);
	iDynTree::MatrixDynSize xbc_hat_dot(3,3);
	iDynTree::MatrixDynSize Jbc_dot(6,6+n);
	iDynTree::Vector3 xbo_dot;
	
	 
	//Set ausiliary matrices
	
	iDynTree::Vector3 xbc_dot;
	
	// Compute T matrix
	    // Get jacobians of the floating base and of the com
	    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
	    kinDynComp.getCenterOfMassJacobian(Jcom);

	    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
	    toEigen(Jbc)<<toEigen(Jcom).block<3,18>(0,6)-toEigen(Jb).block<3,18>(0,6);
	
	    // Get xb (floating base position) and xc ( com position)
	    iDynTree::Position xb = world_H_base.getPosition();
	    iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();
	
	    // Vector xcb=xc-xb
	    toEigen(xbc)=toEigen(xc)-toEigen(xb);
	
	    // Skew of xcb
	    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
	                      toEigen(xbc)[2], 0, -toEigen(xbc)[0],                          
	                     -toEigen(xbc)[1], toEigen(xbc)[0], 0;
	
	Eigen::Matrix<double,6,6> X;
	  X<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), 
         Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);
	  	
	

      Eigen::MatrixXd Mb_Mj= toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(toEigen(MassMatrix).block(0,6,6,18));
	  Eigen::Matrix<double,6,18> Js=X*Mb_Mj;

		
		
     	// Matrix T for the transformation
	    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), Js.block(0,0,3,18),
	                Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Js.block(3,0,3,18),
	                Eigen::MatrixXd::Zero(18,3),  Eigen::MatrixXd::Zero(18,3), Eigen::MatrixXd::Identity(18,18);

		
	
	//Compute time derivative of T 
       // Compute derivative of xbc
	   toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
	   Eigen::VectorXd  mdr=robot_mass*toEigen(xbc_dot);
       Eigen::Matrix<double,3,3> mdr_hat;
	   mdr_hat<<0, -mdr[2], mdr[1],
	            mdr[2], 0, -mdr[0],                          
	           -mdr[1], mdr[0], 0;

	   //Compute skew of xbc
   	   toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
	                      toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
	                      -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

	   Eigen::Matrix<double,6,6> dX;
       dX<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot).transpose(),
                      Eigen::MatrixXd::Zero(3,6);
	   // Time derivative of Jbc
	   kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);
	   
	   Eigen::Matrix<double,6,6> dMb;
	   dMb<<Eigen::MatrixXd::Zero(3,3), mdr_hat.transpose(),
	        mdr_hat, Eigen::MatrixXd::Zero(3,3);
	  
       Eigen::MatrixXd inv_dMb1=(toEigen(MassMatrix).block(0,0,6,6).transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dMb.transpose())).transpose();
	   Eigen::MatrixXd inv_dMb2=-(toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( inv_dMb1));
     
	   Eigen::Matrix<double,6,18> dJs=dX*Mb_Mj+X*inv_dMb2*toEigen(MassMatrix).block(0,6,6,18);
	   

	  //TDMAT Tj(toEigen(qb),toEigen(dqb));
	 // Eigen::Matrix<double,3,12> Jbcdot=Tj.getTdmatrix();
       /*std::cout<<"dMb"<<dMb<<std::endl;
	    std::cout<<"dJ"<<toEigen(MassMatrix).block(0,0,6,6)<<std::endl;
      std::cout<<"dJ"<<inv_dMb2<<std::endl;
	  std::cout<<"dmbmj"<<Mb_Mj<<std::endl;
	  std::cout<<"Mbj"<<toEigen(MassMatrix).block(0,6,6,12)<<std::endl;
	  std::cout<<"dx"<<dX<<std::endl;
	  std::cout<<"x"<<X<<std::endl;
	  std::cout<<"dJ"<<-dJs<<std::endl;*/
	// Tdot matrix
	   toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), -dJs.block(0,0,3,18),
                      Eigen::MatrixXd::Zero(21,24);
	// Compute T matrix
	    // Get jacobians of the floating base and of the objectframe
	  /*  kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
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
  
Eigen::Matrix<double,3,3> QUADRUPED::getBRworldtransform()
{    
	
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("back_right_foot");
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getBLworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("back_left_foot");

   return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getFLworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("front_left_foot");

 return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> QUADRUPED::getFRworldtransform()
{   iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("front_right_foot");

   return toEigen(World_br.getRotation());
}


Eigen::MatrixXd QUADRUPED::getOBJpos()
{
  Eigen::Matrix<double,6,1> pos;
  double roll;
  double pitch;
  double yaw;
  obj_->getFrame().M.GetRPY(roll,pitch,yaw);
  pos<< toEigenKDL(obj_->getFrame().p),
        roll,
		pitch,
		yaw;
	
 return pos;
}
    
Eigen::MatrixXd QUADRUPED::getCOMvel()
{

 return toEigen(CoM_vel);

}

Eigen::MatrixXd QUADRUPED::getBRpos()
{
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("back_right_foot");
	return toEigen(World_br.getPosition());

}

Eigen::MatrixXd QUADRUPED::getBLpos()
{
	iDynTree::Transform  World_bl;
    World_bl=kinDynComp.getWorldTransform("back_left_foot");
	return toEigen(World_bl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFLpos()
{
	iDynTree::Transform  World_fl;
    World_fl=kinDynComp.getWorldTransform("front_left_foot");
	return toEigen(World_fl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFRpos()
{
	iDynTree::Transform  World_fr;
    World_fr=kinDynComp.getWorldTransform("front_right_foot");
	return toEigen(World_fr.getPosition());

}

Eigen::MatrixXd QUADRUPED::getBRvel()
{   iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("back_right_foot");

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getBLvel()
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("back_left_foot");

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFLvel()
{
iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("front_left_foot");

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFRvel()
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("front_right_foot");

	return toEigen(br_vel.getLinearVec3() );

}


Eigen::MatrixXd QUADRUPED::getOBJvel()
{
  Eigen::Matrix<double,6,1> vel;
  vel<< toEigenKDL(obj_->getSpatialVelocity().vel),
        toEigenKDL(obj_->getSpatialVelocity().rot);
   
  
 return vel;

}

double QUADRUPED::getMass()
{
	return robot_mass;
}

Eigen::MatrixXd QUADRUPED::getsolution()
{
   
	return x_eigen;
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
Eigen::MatrixXd QUADRUPED::qpproblem( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::MatrixXd vdotswdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot)
{

// Set variables
   int variables=52;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(132,53);
   Lt.setlength(132);



   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
 
     ////////////////////////////////////
   Eigen::Matrix<double, 6,12> G_=obj_->getGraspMatrix();
    std::cout<<"grasp"<<G_<<std::endl;
    // compute object wrench in body frame
    KDL::Wrench F_b = obj_->computeID();

    double cost;
    // e.block(3,0,3,1) = tiltingTorque(cost, 2, 1e-2);
 double Kp = 250, Kd = 50;
   // std::cout << "F_b ID: " << toEigen(F_b).transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> y = obj_->getMassMatrix()*(spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot));
    std::cout << "y " << spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc+ spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot) << std::endl;
				 //Eigen::Matrix<double, 6, 1> y1 = obj_->getMassMatrix()*(vdotswdes);
     std::cout << "y1" << obj_->getMassMatrix()<< std::endl;
    F_b = F_b + toKDLWrench(y);

   
   Eigen::Matrix<double,6,24> Jobj_b=b_J_obj_.data*toEigen(T).inverse();

   Eigen::Matrix<double,24,24> P=Eigen::Matrix<double,24,24>::Identity()-toEigen(JacCOM_lin).transpose()*(toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse()*toEigen(JacCOM_lin).transpose()).inverse()*toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse();

   
   Eigen::Matrix<double,24,24> Mc=P*toEigen(MassMatrixCOM)+Eigen::Matrix<double,24,24>::Identity()-P;

   Eigen::Matrix<double,6,6> Lambda=(Jobj_b*Mc.inverse()*P*Jobj_b.transpose()).inverse();
  
   Eigen::Matrix<double,6,1> Jdqdobj=b_J_dot_obj_.data*toEigen(dq)+Jobj_b*toEigen(T_inv_dot)*toEigen(dq);

   Eigen::MatrixXd Jc_psinv= toEigen(JacCOM_lin).completeOrthogonalDecomposition().pseudoInverse();

   Eigen::Matrix<double,6,1> B_term=Lambda*(Jobj_b*Mc.inverse()*(P*toEigen(BiasCOM)+Jc_psinv*toEigen(JdqdCOM_lin))-Jdqdobj);

   Eigen::Matrix<double, 6, 1> y1 = Lambda*(_desObjAcc)+Kp*e+Kd*edot+B_term;
	/////////////////////////////////////////
   Eigen::Matrix<double, 6, 24> Jswcom=b_J_obj_.data*toEigen(T).inverse();
   Eigen::Matrix<double, 6, 24> Jswj=b_J_obj_.data*toEigen(T).inverse();

    std::cout<<"jswcom"<<Jswcom<<std::endl;
   Eigen::Matrix<double, 12, 18> Jstj= toEigen(JacCOM_lin).block(0,6,12,18);

   // cost function quadratic matrix
   Eigen::Matrix<double,12,52> Sigma= Eigen::Matrix<double,12,52>::Zero();
   Sigma.block(0,24,12,12)= Eigen::Matrix<double,12,12>::Identity();


  Eigen::Matrix<double,12,16> F_c_hat;
   F_c_hat.block(0,0,3,4) = contacts_.at(0).getConeVersors();
   F_c_hat.block(3,4,3,4) = contacts_.at(0).getConeVersors();
   F_c_hat.block(6,8,3,4) = contacts_.at(0).getConeVersors();
   F_c_hat.block(9,12,3,4) = contacts_.at(0).getConeVersors();
	
   Eigen::Matrix<double,16,52> Sigma1= Eigen::Matrix<double,16,52>::Zero();
   Sigma1.block(0,36,16,16)= Eigen::Matrix<double,16,16>::Identity();
   Eigen::Matrix<double,6,52>  T_s1= G_*F_c_hat*Sigma1;

   Eigen::Matrix<double, 6, 12> Jot=toEigen(LambdaObj)*toEigen(Jobj)*toEigen(MassMatrix).inverse()*toEigen(Jac_lin).transpose();
  // Eigen::Matrix<double,6,39>  T_s= Jot*Sigma;
   Eigen::Matrix<double,12,52>  T_s;
   T_s<< Jstcom.transpose()*Sigma,
         T_s1;
   Eigen::Matrix<double,12,12> eigenQ1= 50*Eigen::Matrix<double,12,12>::Identity();
   eigenQ1.block(6,6,6,6)<< 1000*Eigen::Matrix<double,6,6>::Identity();
   eigenQ1.block(9,9,3,3)<< 10000*Eigen::Matrix<double,3,3>::Identity();
   Eigen::Matrix<double,52,52> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,52,52> eigenR= Eigen::Matrix<double,52,52>::Identity();
   //eigenR.block(36,36,6,6)=1000*Eigen::Matrix<double,6,6>::Identity();
   
   Eigen::Matrix<double,52,52> eigenQ= eigenQ2+eigenR;
   //Eigen::Matrix<double,39,39> eigenQ= eigenQ2+Eigen::Matrix<double,39,39>::Identity();
 
   std::cout<<"jswcom1"<<std::endl;
	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
   
	Eigen::Matrix<double,6,1> C_=obj_->getCMatrix();
    Eigen::Matrix<double,6,1> N_=obj_->getNMatrix();
	Eigen::Matrix<double,12,1> W_des;
	W_des<<Wcom_des,
	       toEigenKDL(F_b); 
	Eigen::Matrix<double,52,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*W_des;

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }





    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(52,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	std::cout << "y1" << vdotswdes<< std::endl;
 
   //Equality constraints
   Eigen::Matrix<double,24, 52> eigenA= Eigen::Matrix<double,24, 52>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,24,6,12)=-Jstcom.transpose();

	eigenA.block(0,36,6,16)=-G_*F_c_hat;

    eigenA.block(6,0,12,6)=Jstcom;

    eigenA.block(6,6,12,18)=Jstj;

    eigenA.block(18,0,6,24) = Jobj_b;//obj_->getMassMatrix()*
    //eigenA.block(18,36,6,16) = obj_->getMassMatrix().inverse()*G_*F_c_hat;
     

    // Known term
    Eigen::Matrix<double,24, 1> eigenb= Eigen::Matrix<double,24,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);;

	eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);

    eigenb.block(18,0,6,1)=-Jdqdobj+obj_->getMassMatrix().inverse()*(-C_ + N_ + toEigenKDL(F_b));

    std::cout<<"eigenb"<<eigenb.block(18,0,6,1)<<std::endl;
	std::cout<<"eigenA"<<eigenA.block(18,0,6,52)<<std::endl;
    

	//Inequality Constraints

	Eigen::Matrix<double,108, 52> eigenD= Eigen::Matrix<double,108,52>::Zero();
	
	 // Torque limits
	eigenD.block(20,6,18,18)=toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(20,24,18,12)=-Jstj.transpose();

	eigenD.block(20,36,6,16)=-G_*F_c_hat;

	eigenD.block(38,6,18,18)=-toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(38,24,18,12)=Jstj.transpose();

	eigenD.block(38,36,6,16)=G_*F_c_hat;

	/*eigenD.block(56,0,6,6)=Jswcom.block(0,0,6,6);

    eigenD.block(56,6,6,18)=Jswj.block(0,6,6,18);

	eigenD.block(56,36,6,6)=-Eigen::Matrix<double,6,6>::Identity();

	eigenD.block(62,0,6,6)=-Jswcom.block(0,0,6,6);

    eigenD.block(62,6,6,18)=-Jswj.block(0,6,6,18);

    eigenD.block(62,36,6,6)=-Eigen::Matrix<double,6,6>::Identity();

	eigenD.block(68,6,18,18)=Eigen::Matrix<double,18,18>::Identity();

    eigenD.block(86,6,18,18)=-Eigen::Matrix<double,18,18>::Identity();*/

	
	eigenD.block(56,6,18,18)=Eigen::Matrix<double,18,18>::Identity();

    eigenD.block(74,6,18,18)=-Eigen::Matrix<double,18,18>::Identity();
    
	eigenD.block(92,36,16,16) = -Eigen::Matrix<double,16,16>::Identity();

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
     
	    Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
	

    eigenD.block(0,24,20,12)=Dfr;

	//eigenD.block(92,36,20,12)=Dfr;


    // Known terms for inequality
	Eigen::Matrix<double,108, 1> eigenC= Eigen::Matrix<double,108,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,18,1> tau_max=60*Eigen::Matrix<double,18,1>::Ones();
	Eigen::Matrix<double,18,1> tau_min=-60*Eigen::Matrix<double,18,1>::Ones();

    Eigen::Matrix<double,18, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,18,1);


    	for (int i=0; i<4; i++)
	{ eigenC.block(4+i*5,0,1,1)<<-2;}
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

	//eigenC.block(68,0,18,1)=ddqmax;
	 //eigenC.block(86,0,18,1)=-ddqmin;
	
    eigenC.block(56,0,18,1)=ddqmax;
	 eigenC.block(74,0,18,1)=-ddqmin;

	// eigenC.block(92,0,16,1)<<0,0,0,-1.5,0,0,0,-1.5,0,0,0,-1.5,0,0,0,-1.5;


	 

	
	 iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(kinDynComp.getFrameIndex("wrist_flexion_link")); 
	 Eigen::Matrix<double,6,1> Jdqdsw=b_J_dot_obj_.data*toEigen(T).inverse()*toEigen(dq)+Jswcom*toEigen(T_inv_dot)*toEigen(dq);

     /*eigenC.block(56,0,6,1)=y-Jdqdsw.block(0,0,6,1);
	 eigenC.block(62,0,6,1)=-y+Jdqdsw.block(0,0,6,1);*/

	//eigenC.block(56,0,6,1)=vdotswdes-Jdqdsw.block(0,0,6,1);
	//eigenC.block(62,0,6,1)=-vdotswdes+Jdqdsw.block(0,0,6,1);
	
     //Linear constraints matrix
    Eigen::Matrix<double,132, 53> eigenL= Eigen::Matrix<double,132, 53>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 24)
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
    

    
    // Set qp
   // alglib::minqpsetbc(state, bl, bu);
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	//alglib::minqpsetalgodenseipm(state, 1.0e-9);
     alglib::minqpsetalgodenseaul(state, 1.0, 1.0e+2, 1);

    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

   
	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);

     std::cout<<"solution "<<x_eigen<<std::endl;
    
     Eigen::Matrix<double,6,1> Wobt=Jot*x_eigen.block(24,0,12,1);

	
	std::vector<Eigen::Matrix<double,3,24>> Jci = b_J_c_;


    Eigen::Matrix<double,12,24> Jc = toEigenKDL(Jci);

    Eigen::Matrix<double,12,1> Fc;
    Fc= F_c_hat*x_eigen.block(36,0,16,1);

      std::cout<<"Fc"<<Fc<<std::endl;
    Eigen::Matrix<double,6,1> Fb_des=G_*F_c_hat*x_eigen.block(36,0,16,1);
	std::cout<<"Fb"<<Fb_des<<std::endl;
	std::cout<<"Fb"<<toEigenKDL(F_b)<<std::endl;
	Eigen::Matrix<double,18,1> tau= Eigen::Matrix<double,18,1>::Zero();
	tau=toEigen(MassMatrixCOM).block(6,6,18,18)*x_eigen.block(6,0,18,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(24,0,12,1)+Jobj_b.block(0,6,6,18).transpose()*Fb_des;//-Jc.block(0,6,12,18).transpose()*Fc;//;
	return tau;

}

// Quadratic problem back right leg
Eigen::VectorXd QUADRUPED::qpproblembr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::VectorXd vdotswdes,  SWING_LEGS swinglegs, Eigen::MatrixXd vdotobjdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot)
{
	int swl1, swl2, stl1, stl2;
    switch(swinglegs){
		case L1: swl1=0;
		swl2=0 ; 
		stl1=0;
		stl2=0 ; 
		break;
		case L2: swl1=0;
		swl2=6 ;
		stl1=3;
		stl2=9 ; 
		 break;
		case L3: swl1=3;
		swl2=9;
		stl1=0;
		stl2=6; 
		 break;
	}

	// Set variables
   int variables=52;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(128,53);
   Lt.setlength(128);



      ////////////////////////////////////
   Eigen::Matrix<double, 6,12> G_=obj_->getGraspMatrix();
    std::cout<<"grasp"<<G_<<std::endl;
    // compute object wrench in body frame
    KDL::Wrench F_b = obj_->computeID();

    double cost;
    // e.block(3,0,3,1) = tiltingTorque(cost, 2, 1e-2);
 double Kp = 250, Kd = 50;
   // std::cout << "F_b ID: " << toEigen(F_b).transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> y = obj_->getMassMatrix()*(spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot));
    std::cout << "y " << spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc+ spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot) << std::endl;
				// Eigen::Matrix<double, 6, 1> y1 = obj_->getMassMatrix()*(vdotswdes);
     //std::cout << "y1" << vdotswdes<< std::endl;
    F_b = F_b + toKDLWrench(y);

   
   Eigen::Matrix<double,6,24> Jobj_b=b_J_obj_.data*toEigen(T).inverse();

   Eigen::Matrix<double,24,24> P=Eigen::Matrix<double,24,24>::Identity()-toEigen(JacCOM_lin).transpose()*(toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse()*toEigen(JacCOM_lin).transpose()).inverse()*toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse();

   
   Eigen::Matrix<double,24,24> Mc=P*toEigen(MassMatrixCOM)+Eigen::Matrix<double,24,24>::Identity()-P;

   Eigen::Matrix<double,6,6> Lambda=(Jobj_b*Mc.inverse()*P*Jobj_b.transpose()).inverse();
  
   Eigen::Matrix<double,6,1> Jdqdobj=b_J_dot_obj_.data*toEigen(T).inverse()*toEigen(dq)+Jobj_b*toEigen(T_inv_dot)*toEigen(dq);

   Eigen::MatrixXd Jc_psinv= toEigen(JacCOM_lin).completeOrthogonalDecomposition().pseudoInverse();

   Eigen::Matrix<double,6,1> B_term=Lambda*(Jobj_b*Mc.inverse()*P*toEigen(BiasCOM)-Jdqdobj+Jobj_b*Mc.inverse()*Jc_psinv*toEigen(JdqdCOM_lin));


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 6, 6> Jstcom= Eigen::Matrix<double,6,6>::Zero();
    Jstcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(stl1,0,3,6);
	Jstcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(stl2,0,3,6);

   Eigen::Matrix<double, 6, 18> Jstj= Eigen::Matrix<double,6,18>::Zero();
    Jstj.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,6,3,18);
    Jstj.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,6,3,18);

   
  Eigen::Matrix<double, 6, 18> Jst= Eigen::Matrix<double,6,18>::Zero();
 Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
    Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);

   Eigen::Matrix<double, 6, 6> Jswcom= Eigen::Matrix<double,6,6>::Zero();
    Jswcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(swl1,0,3,6);
	 Jswcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(swl2,0,3,6);
   Eigen::Matrix<double, 6, 18> Jswj=  Eigen::Matrix<double,6,18>::Zero();
   Jswj.block(0,0,3,18)=toEigen(JacCOM_lin).block(swl1,6,3,18);
   Jswj.block(3,0,3,18)=toEigen(JacCOM_lin).block(swl2,6,3,18);



       Eigen::Matrix<double, 6, 24> Jobj_=b_J_obj_.data*toEigen(T).inverse();

   // cost function quadratic matrix
   Eigen::Matrix<double,6,52> Sigma= Eigen::Matrix<double,6,52>::Zero();
   Sigma.block(0,24,6,6)= Eigen::Matrix<double,6,6>::Identity();

        Eigen::Matrix<double,12,16> F_c_hat;
   F_c_hat.block(0,0,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(3,4,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(6,8,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(9,12,3,4) = contacts_.at(0).getConeVersors();

   Eigen::Matrix<double,16,52> Sigma1= Eigen::Matrix<double,16,52>::Zero();
   Sigma1.block(0,36,16,16)= Eigen::Matrix<double,16,16>::Identity();
   Eigen::Matrix<double,6,52>  T_s1= G_*F_c_hat*Sigma1;
   
   Eigen::Matrix<double,6,52>  T_s= Jstcom.transpose()*Sigma;
   //Eigen::Matrix<double,6,42>  T_s2= Jobj_*Sigma2;
   Eigen::Matrix<double,12,52>  T_s3;
   T_s3<<T_s,
        T_s1;

   Eigen::Matrix<double,12,12> eigenQ1= 50*Eigen::Matrix<double,12,12>::Identity();
    eigenQ1.block(6,6,6,6)<< 1000*Eigen::Matrix<double,6,6>::Identity();
   eigenQ1.block(9,9,3,3)<< 10000*Eigen::Matrix<double,3,3>::Identity();

   Eigen::Matrix<double,52,52> eigenQ2= T_s3.transpose()*eigenQ1*T_s3;
   
   Eigen::Matrix<double,52,52> eigenR= Eigen::Matrix<double,52,52>::Identity();
   eigenR.block(30,30,6,6)=1000*Eigen::Matrix<double,6,6>::Identity();
   eigenR.block(36,36,6,6)=1000*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,52,52> eigenQ= eigenQ2+eigenR;
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	 iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(kinDynComp.getFrameIndex("wrist_flexion_link")); 
//	 Eigen::Matrix<double,6,1> Jdqdobj=s_J_dot_obj_.data*toEigen(dq)+Jobj_*toEigen(T_inv_dot)*toEigen(dq);
	  // cost function linear matrix
    //Eigen::Matrix<double,6,24> Jobj_b=b_J_obj_.data*toEigen(T).inverse();
	Eigen::Matrix<double,6,1> C_=obj_->getCMatrix();
    Eigen::Matrix<double,6,1> N_=obj_->getNMatrix();
	Eigen::Matrix<double,12,1> W_des;
	W_des<<Wcom_des,
	       toEigenKDL(F_b); 

	Eigen::Matrix<double,52,1> eigenc= -T_s3.transpose()*eigenQ1.transpose()*W_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }




    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(52,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	 std::cout<<"prova1"<<std::endl;

	//Equality constraints
	Eigen::Matrix<double,18, 52> eigenA= Eigen::Matrix<double,18, 52>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,24,6,6)=-Jstcom.transpose();

    eigenA.block(0,36,6,16)=-G_*F_c_hat;

    eigenA.block(6,0,6,6)=Jstcom;

    eigenA.block(6,6,6,18)=Jstj;

	 eigenA.block(12,0,6,24) = Jobj_b;
    //eigenA.block(12,36,6,16) = obj_->getMassMatrix().inverse()*G_*F_c_hat;

	//std::cout<<"eigenA"<<eigenA<<std::endl;
std::cout<<"prova2"<<std::endl;
    // Known term
    Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();

    Eigen::Matrix<double,6,1> Jdqdst= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(stl2,0,3,1);
			 
    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;


	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);

	eigenb.block(6,0,6,1)=-Jdqdst;

	eigenb.block(12,0,6,1)=-Jdqdobj+obj_->getMassMatrix().inverse()*(-C_ + N_ + toEigenKDL(F_b));

    //std::cout<<"eigenB"<<eigenb<<std::endl;
    
   
	//Inequality Constraints

	Eigen::Matrix<double,110,52> eigenD= Eigen::Matrix<double,110,52>::Zero();
	
	 // Torque limits
	eigenD.block(10,6,18,18)=toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(10,24,18,6)=-Jstj.transpose();

	eigenD.block(10,36,6,16)=-G_*F_c_hat;

	eigenD.block(28,6,18,18)=-toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(28,24,18,6)=Jstj.transpose();

	eigenD.block(28,36,6,16)=G_*F_c_hat;
    
    eigenD.block(46,0,3,6)=Jswcom.block(0,0,3,6);

    eigenD.block(46,6,3,18)=Jswj.block(0,0,3,18);

	eigenD.block(49,0,3,6)=Jswcom.block(3,0,3,6);

    eigenD.block(49,6,3,18)=Jswj.block(3,0,3,18);

	eigenD.block(46,30,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(49,33,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(52,0,3,6)=-Jswcom.block(0,0,3,6);

    eigenD.block(52,6,3,18)=-Jswj.block(0,0,3,18);

	eigenD.block(55,0,3,6)=-Jswcom.block(3,0,3,6);

    eigenD.block(55,6,3,18)=-Jswj.block(3,0,3,18);

    eigenD.block(52,30,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(55,33,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(58,6,18,18)=Eigen::Matrix<double,18,18>::Identity();

    eigenD.block(76,6,18,18)=-Eigen::Matrix<double,18,18>::Identity();

	/*eigenD.block(94,0,6,6)=Jobj_.block(0,0,6,6);

    eigenD.block(94,6,6,18)=Jobj_.block(0,6,6,18);

	eigenD.block(94,36,6,6)=-Eigen::Matrix<double,6,6>::Identity();

	eigenD.block(100,0,6,6)=-Jobj_.block(0,0,6,6);

    eigenD.block(100,6,6,18)=-Jobj_.block(0,6,6,18);

    eigenD.block(100,36,6,6)=-Eigen::Matrix<double,6,6>::Identity();*/

	eigenD.block(94,36,16,16) = -Eigen::Matrix<double,16,16>::Identity();
    
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
     
	    Eigen::Matrix<double,10,6> Dfr=Eigen::Matrix<double,10,6>::Zero();

		for(int i=0; i<2; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,24,10,6)=Dfr;

    //std::cout<<"eigenD"<<eigenD<<std::endl;
    // Known terms for inequality
	Eigen::Matrix<double,110, 1> eigenC= Eigen::Matrix<double,110,1>::Zero();

	// Torque limits
    Eigen::Matrix<double,18,1> tau_max=60*Eigen::Matrix<double,18,1>::Ones();
	tau_max.block(0,0,1,1)<<100;
	tau_max.block(13,0,5,1)<<100*Eigen::Matrix<double,5,1>::Ones();
	Eigen::Matrix<double,18,1> tau_min=-60*Eigen::Matrix<double,18,1>::Ones();
    tau_min.block(0,0,1,1)<<-100;
	tau_min.block(13,0,5,1)<<-100*Eigen::Matrix<double,5,1>::Ones();
    Eigen::Matrix<double,18, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,18,1);

    
	eigenC.block(10,0,18,1)=tau_max-eigenBiascom;
	eigenC.block(28,0,18,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,18, 1> eigenq=toEigen(q).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigendq=toEigen(dq).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,18, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,18, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,18, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(58,0,18,1)=ddqmax;
	 eigenC.block(76,0,18,1)=-ddqmin;


	 Eigen::Matrix<double,6,1> Jdqdsw= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(swl2,0,3,1);
 


	 eigenC.block(46,0,6,1)= vdotswdes-Jdqdsw;
	 eigenC.block(52,0,6,1)= -vdotswdes+Jdqdsw;
	
	//  iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(23); 
	/*Eigen::Matrix<double,6,1> Jdqdobj=s_J_dot_obj_.data*toEigen(T).inverse()*toEigen(dq)+Jobj_*toEigen(T_inv_dot)*toEigen(dq);

	 eigenC.block(94,0,6,1)=vdotobjdes-Jdqdobj.block(0,0,6,1);
	 eigenC.block(100,0,6,1)=-vdotobjdes+Jdqdobj.block(0,0,6,1);*/
	 
	 
	  // std::cout<<"eigenc"<<eigenC<<std::endl;
     //Linear constraints matrix
    Eigen::Matrix<double,128, 53> eigenL= Eigen::Matrix<double,128,53>::Zero();

   
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
    

     
    
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0, 1.0e+2, 1);
    

    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

   

	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);


    std::cout<<"solution"<<x_eigen<<std::endl;

	
	std::vector<Eigen::Matrix<double,3,24>> Jci = b_J_c_;


    Eigen::Matrix<double,12,24> Jc = toEigenKDL(Jci);

    Eigen::Matrix<double,12,1> Fc;
    Fc= F_c_hat*x_eigen.block(36,0,16,1);
        std::cout<<"Fc"<<Fc<<std::endl;
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(18);
	//Eigen::Matrix<double,6,1> Fb_des=obj_->getMassMatrix()*Jobj_b*x_eigen.block(0,0,24,1)+obj_->getMassMatrix()*s_J_dot_obj_.data*toEigen(dq)+C_-N_;
	Eigen::Matrix<double,6,1> Fb_des=G_*F_c_hat*x_eigen.block(36,0,16,1);
	std::cout<<"Fb"<<Fb_des<<std::endl;
	std::cout<<"Fb"<<toEigenKDL(F_b)<<std::endl;
	tau=toEigen(MassMatrixCOM).block(6,6,18,18)*x_eigen.block(6,0,18,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(24,0,6,1)+Jobj_b.block(0,6,6,18).transpose()*Fb_des;//-Jc.block(0,6,12,18).transpose()*Fc;//+Jobj_b.block(0,6,6,18).transpose()*Fb_des;//-Jobj_b.block(0,6,6,18).transpose()*Fb_des;

		std::cout<<"acc"<<toEigen(JacCOM_lin).block(swl1,0,3,24)*x_eigen.block(0,0,24,1)+toEigen(JdqdCOM_lin).block(swl1,0,3,1)<<std::endl;
				std::cout<<"acc"<<toEigen(JacCOM_lin).block(swl2,0,3,24)*x_eigen.block(0,0,24,1)+toEigen(JdqdCOM_lin).block(swl2,0,3,1)<<std::endl;
	std::cout<<"vel"<<toEigen(JacCOM_lin).block(swl1,0,3,24)*toEigen(dq)<<std::endl;
	std::cout<<"wobt"<<Jst.transpose()*x_eigen.block(24,0,6,1)<<std::endl;
	return tau;

}

Eigen::VectorXd QUADRUPED::qpproblemol( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes,  SWING_LEG swingleg, Eigen::MatrixXd vdotobjdes, Eigen::MatrixXd _desObjAcc,  Eigen::MatrixXd e, Eigen::MatrixXd edot)
{
	int swl1, stl1, stl2, stl3;
    switch(swingleg){
		case BR: swl1=0; 
		stl1=3;
		stl2=6 ; 
		stl3=9;
		break;
		case FL: swl1=6;
		stl1=0;
		stl2=3 ; 
		stl3=9;
		break;
		case BL: swl1=3;
		stl1=0;
		stl2=6; 
		stl3=9;
		break;
		case FR: swl1=9;
		stl1=0;
		stl2=3; 
		stl3=6;
		 break;
	}

	// Set variables
   int variables=52;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(130,53);
   Lt.setlength(130);

      ////////////////////////////////////
   Eigen::Matrix<double, 6,12> G_=obj_->getGraspMatrix();
    std::cout<<"grasp"<<G_<<std::endl;
    // compute object wrench in body frame
    KDL::Wrench F_b = obj_->computeID();

    double cost;
    // e.block(3,0,3,1) = tiltingTorque(cost, 2, 1e-2);
 double Kp = 250, Kd = 50;
   // std::cout << "F_b ID: " << toEigen(F_b).transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> y = obj_->getMassMatrix()*(spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot));
    std::cout << "y " << spatialRotation(obj_->getFrame().M.Inverse())*_desObjAcc+ spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot) << std::endl;
				// Eigen::Matrix<double, 6, 1> y1 = obj_->getMassMatrix()*(vdotswdes);
     //std::cout << "y1" << vdotswdes<< std::endl;
    F_b = F_b + toKDLWrench(y);

   
   Eigen::Matrix<double,6,24> Jobj_b=b_J_obj_.data*toEigen(T).inverse();

   Eigen::Matrix<double,24,24> P=Eigen::Matrix<double,24,24>::Identity()-toEigen(JacCOM_lin).transpose()*(toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse()*toEigen(JacCOM_lin).transpose()).inverse()*toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse();

   
   Eigen::Matrix<double,24,24> Mc=P*toEigen(MassMatrixCOM)+Eigen::Matrix<double,24,24>::Identity()-P;

   Eigen::Matrix<double,6,6> Lambda=(Jobj_b*Mc.inverse()*P*Jobj_b.transpose()).inverse();
  
   Eigen::Matrix<double,6,1> Jdqdobj=b_J_dot_obj_.data*toEigen(T).inverse()*toEigen(dq)+Jobj_b*toEigen(T_inv_dot)*toEigen(dq);

   Eigen::MatrixXd Jc_psinv= toEigen(JacCOM_lin).completeOrthogonalDecomposition().pseudoInverse();

   Eigen::Matrix<double,6,1> B_term=Lambda*(Jobj_b*Mc.inverse()*P*toEigen(BiasCOM)-Jdqdobj+Jobj_b*Mc.inverse()*Jc_psinv*toEigen(JdqdCOM_lin));



   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 9, 6> Jstcom= Eigen::Matrix<double,9,6>::Zero();
    Jstcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(stl1,0,3,6);
	Jstcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(stl2,0,3,6);
    Jstcom.block(6,0,3,6)= toEigen(JacCOM_lin).block(stl3,0,3,6);

   Eigen::Matrix<double, 9, 18> Jstj= Eigen::Matrix<double,9,18>::Zero();
    Jstj.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,6,3,18);
    Jstj.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,6,3,18);
    Jstj.block(6,0,3,18)=toEigen(JacCOM_lin).block(stl3,6,3,18);

    Eigen::Matrix<double, 9, 18> Jst= Eigen::Matrix<double,9,18>::Zero();
    Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
    Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);
	Jst.block(6,0,3,18)=toEigen(JacCOM_lin).block(stl3,0,3,18);


   Eigen::Matrix<double, 3, 6> Jswcom= Eigen::Matrix<double,3,6>::Zero();
    Jswcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(swl1,0,3,6);
	
   Eigen::Matrix<double, 3, 18> Jswj=  Eigen::Matrix<double,3,18>::Zero();
   Jswj.block(0,0,3,18)=toEigen(JacCOM_lin).block(swl1,6,3,18);


   // cost function quadratic matrix
   Eigen::Matrix<double, 6, 24> Jobj_=toEigen(Jobj)*toEigen(T).inverse();
   Eigen::Matrix<double,9,52> Sigma= Eigen::Matrix<double,9,52>::Zero();
   Sigma.block(0,24,9,9)= Eigen::Matrix<double,9,9>::Identity();
     Eigen::Matrix<double,16,52> Sigma2= Eigen::Matrix<double,16,52>::Zero();
   Sigma2.block(0,36,16,16)= Eigen::Matrix<double,16,16>::Identity();
   

    Eigen::Matrix<double,12,16> F_c_hat;
   F_c_hat.block(0,0,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(3,4,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(6,8,3,4) = contacts_.at(0).getConeVersors();
     F_c_hat.block(9,12,3,4) = contacts_.at(0).getConeVersors();

   Eigen::Matrix<double,6,52>  T_s= Jstcom.transpose()*Sigma;
   Eigen::Matrix<double,6,52>  T_s2= G_*F_c_hat*Sigma2;
   Eigen::Matrix<double,12,52>  T_s3;
   T_s3<<T_s,
         T_s2;


   Eigen::Matrix<double,12,12> eigenQ1= 50*Eigen::Matrix<double,12,12>::Identity();
    eigenQ1.block(6,6,6,6)<< 1000*Eigen::Matrix<double,6,6>::Identity();
   eigenQ1.block(9,9,3,3)<< 10000*Eigen::Matrix<double,3,3>::Identity();

   Eigen::Matrix<double,52,52> eigenQ2= T_s3.transpose()*eigenQ1*T_s3;
   Eigen::Matrix<double,52,52> eigenR= Eigen::Matrix<double,52,52>::Identity();
   eigenR.block(33,33,3,3)=10000*Eigen::Matrix<double,3,3>::Identity();
   
   Eigen::Matrix<double,52,52> eigenQ= eigenQ2+eigenR;
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

      
    // cost function linear matrix
	 iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc(kinDynComp.getFrameIndex("wrist_flexion_link")); 
	 //Eigen::Matrix<double,6,1> Jdqdobj=toEigen(Jdqd1)+Jobj_*toEigen(T_inv_dot)*toEigen(dq);
	 Eigen::Matrix<double,6,1> C_=obj_->getCMatrix();
    Eigen::Matrix<double,6,1> N_=obj_->getNMatrix();
	 Eigen::Matrix<double,12,1> des_term;
	 des_term<< Wcom_des,
	           toEigenKDL(F_b);


	Eigen::Matrix<double,52,1> eigenc= -T_s3.transpose()*eigenQ1.transpose()*des_term; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }


    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(52,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	

	//Equality constraints
	Eigen::Matrix<double,21, 52> eigenA= Eigen::Matrix<double,21,52>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,24,6,9)=-Jstcom.transpose();

	eigenA.block(0,36,6,16)=-G_*F_c_hat;

    eigenA.block(6,0,9,6)=Jstcom;

    eigenA.block(6,6,9,18)=Jstj;

	eigenA.block(15,0,6,24) = Jobj_b;
	//eigenA.block(15,36,6,16) = obj_->getMassMatrix().inverse()*G_*F_c_hat;

	//std::cout<<"eigenA"<<eigenA<<std::endl;

    // Known term
    Eigen::Matrix<double,21, 1> eigenb= Eigen::Matrix<double,21,1>::Zero();

    Eigen::Matrix<double,9,1> Jdqdst= Eigen::Matrix<double,9,1>::Zero();
	 Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(stl2,0,3,1),
			 toEigen(JdqdCOM_lin).block(stl3,0,3,1);
			 
    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;


	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1);

	eigenb.block(6,0,9,1)=-Jdqdst;

	eigenb.block(15,0,6,1)=-Jdqdobj+obj_->getMassMatrix().inverse()*(-C_ + N_ + toEigenKDL(F_b));

    //std::cout<<"eigenB"<<eigenb<<std::endl;
    
	//Inequality Constraints

	Eigen::Matrix<double,109,52> eigenD= Eigen::Matrix<double,109,52>::Zero();
	
	 // Torque limits
	eigenD.block(15,6,18,18)=toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(15,24,18,9)=-Jstj.transpose();

	eigenD.block(15,36,6,16)=-G_*F_c_hat;

	eigenD.block(33,6,18,18)=-toEigen(MassMatrixCOM).block(6,6,18,18);

    eigenD.block(33,24,18,9)=Jstj.transpose();

	eigenD.block(33,36,6,16)=G_*F_c_hat;
    
    eigenD.block(51,0,3,6)=Jswcom;

    eigenD.block(51,6,3,18)=Jswj;

	eigenD.block(51,33,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(54,0,3,6)=-Jswcom;

    eigenD.block(54,6,3,18)=-Jswj;

    eigenD.block(54,33,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(57,6,18,18)=Eigen::Matrix<double,18,18>::Identity();

    eigenD.block(75,6,18,18)=-Eigen::Matrix<double,18,18>::Identity();

	eigenD.block(92,36,16,16) = -Eigen::Matrix<double,16,16>::Identity();
    
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

    eigenD.block(0,24,15,9)=Dfr;

    //std::cout<<"eigenD"<<eigenD<<std::endl;
    // Known terms for inequality
	Eigen::Matrix<double,109, 1> eigenC= Eigen::Matrix<double,109,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,18,1> tau_max=60*Eigen::Matrix<double,18,1>::Ones();
	Eigen::Matrix<double,18,1> tau_min=-60*Eigen::Matrix<double,18,1>::Ones();

    Eigen::Matrix<double,18, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,18,1);
	for (int i=0; i<3; i++)
	{ eigenC.block(4+i*5,0,1,1)<<-2;}
	eigenC.block(15,0,18,1)=tau_max-eigenBiascom;
	eigenC.block(33,0,18,1)=-(tau_min-eigenBiascom);

      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,18, 1> eigenq=toEigen(q).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigendq=toEigen(dq).block(6,0,18,1);
	 Eigen::Matrix<double,18, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,18, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,18, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,18, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(57,0,18,1)=ddqmax;
	 eigenC.block(75,0,18,1)=-ddqmin;
	 
	 Eigen::Matrix<double,3,1> Jdqdsw= Eigen::Matrix<double,3,1>::Zero();
	 Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1);


	 eigenC.block(51,0,3,1)= vdotswdes-Jdqdsw;
	 eigenC.block(54,0,3,1)= -vdotswdes+Jdqdsw;
	
	  // std::cout<<"eigenc"<<eigenC<<std::endl;
     //Linear constraints matrix
    Eigen::Matrix<double,130, 53> eigenL= Eigen::Matrix<double,130,53>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 21)
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
    

    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	alglib::minqpsetalgodenseaul(state, 1.0, 1.0e+2, 1);
    

    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);


	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);


    std::cout<<"solution"<<x_eigen<<std::endl;

    Eigen::Matrix<double,12,1> Fc;
    Fc= F_c_hat*x_eigen.block(36,0,16,1);
        std::cout<<"Fc"<<Fc<<std::endl;
	
	//Eigen::Matrix<double,6,1> Fb_des=obj_->getMassMatrix()*Jobj_b*x_eigen.block(0,0,24,1)+obj_->getMassMatrix()*s_J_dot_obj_.data*toEigen(dq)+C_-N_;
	Eigen::Matrix<double,6,1> Fb_des=G_*F_c_hat*x_eigen.block(36,0,16,1);
	std::cout<<"Fb"<<Fb_des<<std::endl;
	std::cout<<"Fb"<<toEigenKDL(F_b)<<std::endl;
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(18);
	tau=toEigen(MassMatrixCOM).block(6,6,18,18)*x_eigen.block(6,0,18,1)+eigenBiascom-Jstj.transpose()*x_eigen.block(24,0,9,1)+Jobj_b.block(0,6,6,18).transpose()*Fb_des;
	std::cout<<"acc"<<toEigen(JacCOM_lin).block(swl1,0,3,24)*x_eigen.block(0,0,24,1)+toEigen(JdqdCOM_lin).block(swl1,0,3,1)<<std::endl;
	return tau;

}