#include <cstdlib>
#include <iostream>
// Eigen headers 
#include <Eigen/Core>
#include "dogbot_model/quadruped.h"
#include "dogbot_model/quadruped_control.h"
#include "dogbot_model/kdl_object.h"
// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between 
// iDynTree datastructures
#include <mutex>
#include <cmath>
#include <map>
#include <unistd.h>
#include <unordered_map>
#include <iDynTree/Core/EigenHelpers.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "dogbot_model/traj_planner.h"
#include "dogbot_model/traj_planner_sw.h"
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include "gazebo_msgs/ContactsState.h"
#include "kdl_parser/kdl_parser.hpp"
#include "dogbot_model/utils.h"



#include "iostream"
//#include "../lopt.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>

#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <map>
#include <unistd.h>
#include <unordered_map>
//#include "../topt.h"
#include <angles/angles.h>
#include "geometry_msgs/WrenchStamped.h"
#include <geometry_msgs/PointStamped.h>


#include "gazebo_msgs/ContactsState.h"
#include <mutex>

using namespace std;

  
// Global variables
Eigen::Matrix<double,12,1> Fgrf;
double obj_mass;
Eigen::Matrix3d obj_inertia = Eigen::Matrix3d::Identity();
std::vector<double> obj_pos(6,0.0), obj_vel(6,0.0);
Eigen::Matrix<double,18,1> jnt_pos, jnt_vel, _jnt_pos, _jnt_vel;
Eigen::Matrix<double,6,1>  base_pos,  base_vel;
Eigen::Matrix4d world_H_base;
bool joint_state_available = false, base_state_available = false, contact_br= true, contact_fl= true, contact_fr= true, contact_bl= true, obj_state_available = false;;
Eigen::Vector3d gravity;
gazebo::transport::PublisherPtr pub;
gazebo::msgs::WorldControl stepper;  
trajectory_point trajsw;
trajectory_point trajswang;
  int flag;
QUADRUPEDController* controller_ ;
QUADRUPED* doggo;
trajectory_point traj;
Eigen::Matrix<double,3,1>  force_br, force_bl, force_fl, force_fr;
 Eigen::MatrixXd Kcom;
Eigen::MatrixXd Dcom;
Eigen::VectorXd tau;
towr::SplineHolder solution;
towr::NlpFormulation formulation;
ros::Time begin3, begin32;
ros::Subscriber _eebl_sub , _eefl_sub , _eebr_sub, _eefr_sub;
ros::ServiceClient des_pose_set_state_srv ;
unordered_map<int, string> _id2idname;  
unordered_map<int, int> _id2index;      
unordered_map<int, int> _index2id;


    std::ofstream com_file("com_file.txt");
    std::ofstream com_vel_file("com_vel_file.txt");
    std::ofstream com_des_file("com_des_file.txt");
    std::ofstream com_vel_des_file("com_vel_des_file.txt");
    std::ofstream pbr_des_file("pbr_des_file.txt");
    std::ofstream pbr_file("pbr_file.txt");
    std::ofstream pbl_des_file("pbl_des_file.txt");
    std::ofstream pbl_file("pbl_file.txt");
    std::ofstream pfl_des_file("pfl_des_file.txt");
    std::ofstream pfl_file("pfl_file.txt");
    std::ofstream pfr_des_file("pfr_des_file.txt");
    std::ofstream pfr_file("pfr_file.txt");
    std::ofstream fgr_des_file("fgr_des_file.txt");
    std::ofstream fgr_file("fgr_file.txt");
    std::ofstream fest_file("fest_file.txt");
    std::ofstream fext_file("fext_file.txt");
    std::ofstream tau_file("tau_file.txt");
    std::ofstream joint_file("joint_file.txt");
    std::ofstream qb_file("qb_file.txt");    
    std::ofstream ddqopt_file("ddqopt_file.txt");
    std::ofstream joint_file_vel("joint_file_vel.txt");
    std::ofstream objpos_file("objpos_file.txt");
    std::ofstream objvel_file("objvel_file.txt");
    std::ofstream objposdes_file("objposdes_file.txt");
    std::ofstream objveldes_file("objveldes_file.txt");
    std::ofstream objposcubedes_file("objcubedes_file.txt");




// Control signal
 std_msgs::Float64 front_left_knee_tau_msg, front_right_knee_tau_msg, back_left_knee_tau_msg, back_right_knee_tau_msg, front_left_pitch_tau_msg, front_right_pitch_tau_msg,
       back_left_pitch_tau_msg, back_right_pitch_tau_msg, front_left_roll_tau_msg, front_right_roll_tau_msg, back_left_roll_tau_msg, back_right_roll_tau_msg, shoulder_adduction_tau_msg,
       shoulder_flexion_tau_msg, humerus_rotation_tau_msg, elbow_flexion_tau_msg, wrist_rotation_tau_msg, wrist_flexion_tau_msg;

ros::Publisher front_left_knee_effort_pub , front_right_knee_effort_pub, back_left_knee_effort_pub , back_right_knee_effort_pub, front_left_pitch_effort_pub, front_right_pitch_effort_pub,
 back_left_pitch_effort_pub, back_right_pitch_effort_pub, front_left_roll_effort_pub, front_right_roll_effort_pub, back_left_roll_effort_pub, back_right_roll_effort_pub,      
 shoulder_adduction_effort_pub, shoulder_flexion_effort_pub, humerus_rotation_effort_pub, elbow_flexion_effort_pub, wrist_rotation_effort_pub, wrist_flexion_effort_pub;
  
// Get joints position and velocity
// Get joints position and velocity
void jointStateCallback(const sensor_msgs::JointState & msg) {
 

    if( joint_state_available == false ) {

        for( int i=0; i<18; i++) {
            bool found = false;
            int index = 0;
            while( !found && index <  msg.name.size() ) {
                if( msg.name[index] == _id2idname.at( i )    ) {
                    found = true;

                    _id2index.insert( pair< int, int > ( i, index ));
                    _index2id.insert( pair< int, int > ( index, i ));

                }
                else index++;
            }
        }
    }

    for( int i=0; i<18; i++ ) {
        jnt_pos( i, 0) = msg.position[    _id2index.at(i)    ];
    }

    for( int i=0; i<18; i++ ) {
        jnt_vel( i, 0) = msg.velocity[    _id2index.at(i)    ];
    }
    
   joint_state_available=true;
}

void createrobot(std::string modelFile) {  



      iDynTree::ModelLoader mdlLoader;

    bool ok = mdlLoader.loadModelFromFile(modelFile);

      if( !ok )
      {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return;
      }
   
    // Create a KinDynComputations class from the model
     iDynTree::KinDynComputations kinDynComp;
     ok = kinDynComp.loadRobotModel(mdlLoader.model());

     if( !ok )
     {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return;
     }

    
    
    _id2idname.insert( pair< int, string > ( 0, kinDynComp.getDescriptionOfDegreeOfFreedom(0) ));
    _id2idname.insert( pair< int, string > ( 1, kinDynComp.getDescriptionOfDegreeOfFreedom(1) ));
    _id2idname.insert( pair< int, string > ( 2, kinDynComp.getDescriptionOfDegreeOfFreedom(2) ));
    _id2idname.insert( pair< int, string > ( 3, kinDynComp.getDescriptionOfDegreeOfFreedom(3) ));
    _id2idname.insert( pair< int, string > ( 4, kinDynComp.getDescriptionOfDegreeOfFreedom(4) ));
    _id2idname.insert( pair< int, string > ( 5, kinDynComp.getDescriptionOfDegreeOfFreedom(5) ));
    _id2idname.insert( pair< int, string > ( 6, kinDynComp.getDescriptionOfDegreeOfFreedom(6) ));
    _id2idname.insert( pair< int, string > ( 7, kinDynComp.getDescriptionOfDegreeOfFreedom(7) ));
    _id2idname.insert( pair< int, string > ( 8, kinDynComp.getDescriptionOfDegreeOfFreedom(8) ));
    _id2idname.insert( pair< int, string > ( 9, kinDynComp.getDescriptionOfDegreeOfFreedom(9) ));
    _id2idname.insert( pair< int, string > ( 10, kinDynComp.getDescriptionOfDegreeOfFreedom(10) ));
    _id2idname.insert( pair< int, string > ( 11, kinDynComp.getDescriptionOfDegreeOfFreedom(11) ));
    _id2idname.insert( pair< int, string > ( 12, kinDynComp.getDescriptionOfDegreeOfFreedom(12) ));
    _id2idname.insert( pair< int, string > ( 13, kinDynComp.getDescriptionOfDegreeOfFreedom(13) ));
    _id2idname.insert( pair< int, string > ( 14, kinDynComp.getDescriptionOfDegreeOfFreedom(14) ));
    _id2idname.insert( pair< int, string > ( 15, kinDynComp.getDescriptionOfDegreeOfFreedom(15) ));
    _id2idname.insert( pair< int, string > ( 16, kinDynComp.getDescriptionOfDegreeOfFreedom(16) ));
    _id2idname.insert( pair< int, string > ( 17, kinDynComp.getDescriptionOfDegreeOfFreedom(17) ));



}


// Object callback
void objectStateCallback(const gazebo_msgs::LinkStates & msg)
{
    obj_pos.clear();
    obj_vel.clear();

    obj_pos.push_back(msg.pose[1].position.x);
    obj_pos.push_back(msg.pose[1].position.y);
    obj_pos.push_back(msg.pose[1].position.z);
    obj_pos.push_back(msg.pose[1].orientation.x);
    obj_pos.push_back(msg.pose[1].orientation.y);
    obj_pos.push_back(msg.pose[1].orientation.z);

    obj_vel.push_back(msg.twist[1].linear.x);
    obj_vel.push_back(msg.twist[1].linear.y);
    obj_vel.push_back(msg.twist[1].linear.z);
    obj_vel.push_back(msg.twist[1].angular.x);
    obj_vel.push_back(msg.twist[1].angular.y);
    obj_vel.push_back(msg.twist[1].angular.z);

    obj_state_available = true;
}

// Get base position and velocity
void modelStateCallback(const gazebo_msgs::ModelStates & msg)
{
    base_state_available = true;
    world_H_base.setIdentity();

      //quaternion
         tf::Quaternion q(msg.pose[3].orientation.x, msg.pose[3].orientation.y, msg.pose[3].orientation.z,  msg.pose[3].orientation.w);
         Eigen::Matrix<double,3,3> rot;
         tf::matrixTFToEigen(tf::Matrix3x3(q),rot);
     
      //Roll, pitch, yaw
         double roll, pitch, yaw;
         tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
      //Set base pos (position and orientation)
         base_pos<<msg.pose[3].position.x, msg.pose[3].position.y, msg.pose[3].position.z, roll, pitch, yaw;

      //Set transformation matrix
         world_H_base.block(0,0,3,3)= rot;
         world_H_base.block(0,3,3,1)= base_pos.block(0,0,3,1);
  
      //Set base vel
         base_vel<<msg.twist[3].linear.x, msg.twist[3].linear.y, msg.twist[3].linear.z, msg.twist[3].angular.x, msg.twist[3].angular.y, msg.twist[3].angular.z;


        double t = (ros::Time::now()).toSec();
  std::cout<<"tempo_model"<<t<<std::endl;
}

// Feet callbacks
void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	if(eebr->states.empty()){ 
    contact_br= false;
	}
	else{
		contact_br= true;
   force_br<<eebr->states[0].total_wrench.force.x, eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;
	}
}

void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	if(eefl->states.empty()){ 
    contact_fl= false;
	}
	else{
		contact_fl= true;
  force_fl<<eefl->states[0].total_wrench.force.x, eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;

	}
}

void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
    contact_bl= false;
	}
	else{
		contact_bl= true;
   force_bl<<eebl->states[0].total_wrench.force.x, eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;

	}
}

void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){

	if(eefr->states.empty()){ 
    contact_fr= false;
	}
	else{
		contact_fr= true;
   force_fr<<eefr->states[0].total_wrench.force.x, eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;

	}
}

// Stand Phase
void stand_phase( ros::Rate loop_rate, double duration )
{ while((ros::Time::now()-begin3).toSec() < duration)
   if (joint_state_available && base_state_available)
      {  
        
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         double t = (ros::Time::now()-begin3).toSec();
         double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( t1*1000);
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
         



      Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();

        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

          Eigen::MatrixXd Kpo;
        Kpo=2500*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=500*Eigen::MatrixXd::Identity(6,6);


      // Compute control torque
          Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
         deltaobj<< -0.0001, -0.7480, 0.2789,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;



         Eigen::Vector3d posdescube=obj.block(0,0,3,1);
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");

        //Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
       
       tau = controller_->Cntr(composdes, comveldes, comaccdes,
                                 Kcom, Dcom,accdessw,accdesobj,objvelerr, objposerr  );

       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);
    Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<< Tbr*force_br, Tbl*force_bl,Tfl*force_fl,Tfr*force_fr;
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);
    ////////

    ///////////
        ros::spinOnce();
        loop_rate.sleep();
      }
}


// Swing Phase
void swing_phase( ros::Rate loop_rate, double duration , double duration_prev)
{while ((ros::Time::now()-begin3).toSec() < duration && flag==0 )
    { if (joint_state_available && base_state_available)
      {  

        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin3).toSec();

         double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( (t1)*1000);
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

       Eigen::Matrix<double,6,1> accd;
       if (flag==0)
       {Eigen::Matrix<double,6,1> accd;

      
       accd<< solution.ee_motion_.at(1)->GetPoint(t).a(),
              solution.ee_motion_.at(2)->GetPoint(t).a();

        Eigen::Matrix<double,6,1> posdelta;
        posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos(),
              solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

        Eigen::Matrix<double,6,1> veldelta;
        veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel(),
              solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();
        
        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;
        
    
         Eigen::Matrix<double,12,24> J=doggo->getJacobianCOM_linear();
          Eigen::Matrix<double,6,12> Jcom;
        Jcom.block(0,0,3,12)=J.block(0,6,3,12);
        Jcom.block(3,0,3,12)=J.block(6,6,3,12);


       Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();

         Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
         deltaobj<< -0.0001, -0.7480, 0.2789,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         
         
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;


        Eigen::Vector3d posdescube=obj.block(0,0,3,1);
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
        
       tau = controller_->CntrBr(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L2, accdessw, accdesobj,objvelerr, objposerr  );
                             }
       else if (flag==1)
       {Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(1)->GetPoint(t).a();
       /*tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accd, QUADRUPED::SWING_LEG::BR); */
       }
       else if (flag==2)
       {Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(2)->GetPoint(t).a();
       /*tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accd, QUADRUPED::SWING_LEG::FL);*/ 
       }

      
 
      
      // Compute control torque
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);

    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
      pub->Publish(stepper);
      //
       Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<< Eigen::Matrix<double,3,1>::Zero(), Tbl*force_bl, Eigen::Matrix<double,3,1>::Zero(), Tfr*force_fr;
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    // pub->Publish(stepper);
    ////////
    

        ros::spinOnce();
       if(t>duration-0.05)
       if(contact_br==false && contact_fl==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_fl<<std::endl;}
      else if(contact_br==true && contact_fl==false && t>duration-0.1)
      {flag=2;
       std::cout<<"contact"<<contact_br<<std::endl;}
      else if(contact_br==true && contact_fl==true && t>duration-0.1)
      {flag=3;
       std::cout<<"contact"<<contact_br<<std::endl;
       std::cout<<"contact"<<contact_fl<<std::endl;}
        loop_rate.sleep();

      
    }
    }
}

 void swing_phase2( ros::Rate loop_rate, double duration , double duration_prev)
{while ((ros::Time::now()-begin3).toSec() < duration && flag==0 )
    { if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin3).toSec();

        double t1 = (ros::Time::now()-begin32).toSec();
       int idx=round( (t1)*1000);
      std::cout<<"t"<<t<<std::endl;
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         /*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
         

       Eigen::Matrix<double,6,1> accd;
       if (flag==0)
       {Eigen::Matrix<double,6,1> accd;
       accd<< solution.ee_motion_.at(0)->GetPoint(t).a(),
              solution.ee_motion_.at(3)->GetPoint(t).a();


      Eigen::Matrix<double,6,1> posdelta;
        posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos(),
              solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

        Eigen::Matrix<double,6,1> veldelta;
        veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel(),
              solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();

          Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;
   
        Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();
        
        Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
          deltaobj<< -0.0001, -0.6711, 0.2694,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;


          Eigen::Vector3d posdescube=obj.block(0,0,3,1);
      //   tf::Quaternion qu(obj.block(5,0,1,1), obj.block(4,0,1,1), obj.block(3,0,1,1))
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
      

        // Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
      tau = controller_->CntrBr(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L3, accdessw, accdesobj,objvelerr, objposerr );
                                 std::cout<<"accd"<<accd<<std::endl;  }
       else if (flag==1)
       {Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(1)->GetPoint(t).a();
       /*tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accd, QUADRUPED::SWING_LEG::BR); */
       }
       else if (flag==2)
       {Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(2)->GetPoint(t).a();
       /*tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accd, QUADRUPED::SWING_LEG::FL); */
       }

    
      

      
      // Compute control torque
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);
  
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
      pub->Publish(stepper);
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<< Tbr*force_br,  Eigen::Matrix<double,3,1>::Zero(), Tfl*force_fl,Eigen::Matrix<double,3,1>::Zero();

   

        ros::spinOnce();
       if(t>duration-0.01)
       {  
      if(contact_fr==false && contact_bl==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_bl<<std::endl;}
      else if(contact_fr==true && contact_bl==false && t>duration-0.1)
      {flag=2;
       std::cout<<"contact"<<contact_fr<<std::endl;}
      else if(contact_fr==true && contact_bl==true && t>duration-0.1)
      {flag=2;
       std::cout<<"contact"<<contact_fr<<std::endl;
       std::cout<<"contact"<<contact_bl<<std::endl;}
       }

        loop_rate.sleep();
      }
    }
    }


void swing_phasebr( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin3).toSec() < duration &&  flag==0)
    { 

      if (joint_state_available && base_state_available)
      {   
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         double t = (ros::Time::now()-begin3).toSec();

        
          double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( (t1)*1000);
      
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         /*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();


       Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(1)->GetPoint(t).a();

       Eigen::Matrix<double,3,1> posdelta;
        posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos();

        Eigen::Matrix<double,3,1> veldelta;
        veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel();
        
        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

        Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        

       Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();
        
        Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
          deltaobj<< -0.0001, -0.6711, 0.2694,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;


          Eigen::Vector3d posdescube=obj.block(0,0,3,1);
      //   tf::Quaternion qu(obj.block(5,0,1,1), obj.block(4,0,1,1), obj.block(3,0,1,1))
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
        

    
   
       tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::BR,accdessw, accdesobj,objvelerr, objposerr); 
       


      // Set command message
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);

    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);
     
        Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<< Eigen::Matrix<double,3,1>::Zero(), Tbl*force_bl, Tfl*force_fl, Tfr*force_fr;
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    // pub->Publish(stepper);
    ////////
  

    /////////////////////////
        ros::spinOnce();
        loop_rate.sleep();
         if(t>duration-0.01)
       {  
      if(contact_br==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_br<<std::endl;}}
      
      }
    }
    }

     void swing_phasefl( ros::Rate loop_rate, double duration , double duration_prev)
  { 
    while ((ros::Time::now()-begin3).toSec() < duration &&  flag==0)
    { 
      if (joint_state_available && base_state_available)
      { 
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);
          

        // Time
        double t = (ros::Time::now()-begin3).toSec();

        
          double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( (t1)*1000);
      
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         /*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

       Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(2)->GetPoint(t).a();

       Eigen::Matrix<double,3,1> posdelta;
        posdelta<< solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

        Eigen::Matrix<double,3,1> veldelta;
        veldelta<< solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();

        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

        Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;
        
        Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();
        
        Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
          deltaobj<< -0.0001, -0.6711, 0.2694,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;


          Eigen::Vector3d posdescube=obj.block(0,0,3,1);
      //   tf::Quaternion qu(obj.block(5,0,1,1), obj.block(4,0,1,1), obj.block(3,0,1,1))
        gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
        



  tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                          Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::FL,accdessw, accdesobj,objvelerr, objposerr); 
       

      // Set command message
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);


    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);

       Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<<Tbr*force_br , Tbl*force_bl, Eigen::Matrix<double,3,1>::Zero(), Tfr*force_fr;

  

    /////////////////////////
      
        ros::spinOnce();
        loop_rate.sleep();

         if(t>duration-0.01)
       {  
      if(contact_fl==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_fl<<std::endl;}}
      }
      
    }

    }

    void swing_phasebl( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin3).toSec() < duration &&  flag==0)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin3).toSec();

        
         double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( (t1)*1000);
      
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         /*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
      
       Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(0)->GetPoint(t).a();

       Eigen::Matrix<double,3,1> posdelta;
        posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos();

        Eigen::Matrix<double,3,1> veldelta;
        veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel();
        
        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

        Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;


          Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();
        
        Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
          deltaobj<< -0.0001, -0.6711, 0.2694,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;


          Eigen::Vector3d posdescube=obj.block(0,0,3,1);
      //   tf::Quaternion qu(obj.block(5,0,1,1), obj.block(4,0,1,1), obj.block(3,0,1,1))
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
        



       tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::BL, accdessw, accdesobj,objvelerr, objposerr); 
       


       // Set command message
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);
    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);
     
      Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<<Tbr*force_br , Eigen::Matrix<double,3,1>::Zero(), Tfl*force_fl, Tfr*force_fr;

  

        ros::spinOnce();
        loop_rate.sleep();

         if(t>duration-0.01)
       {  
      if(contact_bl==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_bl<<std::endl;}}
      
      }
    }
    }

    void swing_phasefr( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin3).toSec() < duration &&  flag==0)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin3).toSec();

        
         double t1 = (ros::Time::now()-begin32).toSec();
         int idx=round( (t1)*1000);
      
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         /*toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx), 0,  0, 0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,  0, 0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,  0, 0;*/
         
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
     
         
       Eigen::Matrix<double,3,1> accd;
       accd<< solution.ee_motion_.at(3)->GetPoint(t).a();

       Eigen::Matrix<double,3,1> posdelta;
        posdelta<< solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

        Eigen::Matrix<double,3,1> veldelta;
        veldelta<< solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

        Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;
       
      Eigen::MatrixXd Kpo;
        Kpo=250*Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Kdo;
        Kdo=50*Eigen::MatrixXd::Identity(6,6);

        Eigen::Matrix<double,6,1>  obj= doggo->getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo->getOBJvel();
        
        Eigen::Matrix<double,6,1> accdesobj;
         accdesobj<< trajsw.acc.block(0,idx,3,1),
                      trajswang.acc.block(0,idx,3,1);
         
         //toEigen(comaccdes);

           Eigen::Matrix<double,6,1> veldesobj;
         veldesobj<< trajsw.vel.block(0,idx,3,1),
                      trajswang.vel.block(0,idx,3,1);
         //toEigen(comveldes);

          Eigen::Matrix<double,6,1> deltaobj;
         // deltaobj<< 0.002, -0.6773, 0.25,  0.1900, 0, 0;
          deltaobj<< -0.0001, -0.6711, 0.2694,  0.1900, 0, 0; 
           Eigen::Matrix<double,6,1> posdesobj;
         posdesobj<< trajsw.pos.block(0,idx,3,1),
                     trajswang.pos.block(0,idx,3,1);
         Eigen::Matrix<double,6,1> accdessw=accdesobj+Kdo*(veldesobj-objvel)+Kpo*(posdesobj-obj);
         //toEigen(composdes)+deltaobj;
         Eigen::Matrix<double,6,1> objvelerr=veldesobj-objvel;
         Eigen::Matrix<double,6,1> objposerr=posdesobj-obj;



          Eigen::Vector3d posdescube=obj.block(0,0,3,1);
      //   tf::Quaternion qu(obj.block(5,0,1,1), obj.block(4,0,1,1), obj.block(3,0,1,1))
         gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x =  posdescube[0]; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = posdescube[1]; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = posdescube[2];
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.w = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
        


       tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                 Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::FR, accdessw, accdesobj,objvelerr, objposerr); 
     
      // Set command message
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(17);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(16);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(0);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(1);
       shoulder_flexion_tau_msg.data=tau(11);
       humerus_rotation_tau_msg.data=tau(12);
       elbow_flexion_tau_msg.data=tau(13);
       wrist_rotation_tau_msg.data=tau(14);
       wrist_flexion_tau_msg.data=tau(15);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg); 
    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
     pub->Publish(stepper);

       Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
       Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
       Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
       Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       Fgrf<<Tbr*force_br , Tbl*force_bl, Tfl*force_fl, Eigen::Matrix<double,3,1>::Zero();
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    // pub->Publish(stepper);
    ////////
   

        ros::spinOnce();
        loop_rate.sleep();

         if(t>duration-0.01)
       {  
      if(contact_fr==true && t>duration-0.1)
      {flag=1;
       std::cout<<"contact"<<contact_fr<<std::endl;}}
      
      }
    }
    }




towr::NlpFormulation computetrajecotry(int gait_flag)
   //int gait_flag)
 {   
     std::cout<<"Phase durations"<<std::endl;

    towr::NlpFormulation formulation_;
    // terrain
  formulation_.terrain_ = std::make_shared<towr::FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  Eigen::Matrix<double,6,6> Mcom;
  Mcom<< doggo->getMassMatrixCOM_com();
  

  formulation_.initial_base_.lin.at(towr::kPos) << doggo->getCOMpos().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kPos) << doggo->getCOMpos().block(3,0,3,1);
  formulation_.initial_base_.lin.at(towr::kVel) << doggo->getCOMvel().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kVel) << doggo->getCOMvel().block(3,0,3,1);

    std::cout<<"Phase durations"<<std::endl;

  // Compute trajectory


 /*if((ros::Time::now()).toSec()>50000 && (ros::Time::now()).toSec()<55000)
   { formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot, formulation_.initial_base_.lin.at(towr::kPos)[2]-0.01, 0.06, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;
     formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.024*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
    formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]-0.0;}
  else if((ros::Time::now()).toSec()<50){
    formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.04, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;
 
    formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]-0.05*sin(formulation_.initial_base_.ang.at(towr::kPos)[2]+0.25), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.05*cos(formulation_.initial_base_.ang.at(towr::kPos)[2]+0.025),  formulation_.initial_base_.lin.at(towr::kPos)[2]-0.017;
    formulation_.final_base_.ang.at(towr::kPos) <<  0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]+0.025;
  }*/
   if((ros::Time::now()).toSec()>0 && (ros::Time::now()).toSec()<7 )
    {formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.06, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;

      formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
     formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]-0.025;}
   /*else if((ros::Time::now()).toSec()<7){
     formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.04, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;

     formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04, 0.40229;
     formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
   }*/
   else if((ros::Time::now()).toSec()>7  && (ros::Time::now()).toSec()<14 )
    { formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.06, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;

      formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2];}
   else if((ros::Time::now()).toSec()>14 && (ros::Time::now()).toSec()<27)
    { formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.06, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;

      formulation_.final_base_.lin.at(towr::kPos) <<formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]+0.025;}
   else if((ros::Time::now()).toSec()>27 )
    { formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot,  0.39, 0.06, Mcom(3,3),Mcom(4,4) ,Mcom(5,5) ,Mcom(3,4), Mcom(3,5), Mcom(4,5)) ;

      formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2];}


   auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
  formulation_.initial_ee_W_ = nominal_stance_B;  
  std::cout<<"Phase durations"<<formulation_.initial_ee_W_ .size()<<std::endl;

  Eigen::Vector3d pos_ee;
  for (int ee=0; ee<4; ee++){
    switch(ee){
     case 0: pos_ee=doggo->getBLpos();
     break;
     case 1: pos_ee=doggo->getBRpos();
     break;
     case 2: pos_ee=doggo->getFLpos();
     break;
     case 3: pos_ee=doggo->getFRpos();
     break;
    }

    formulation_.initial_ee_W_.at(ee)[0]=pos_ee[0];
    formulation_.initial_ee_W_.at(ee)[1]=pos_ee[1];

  }

  std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){  p[2]= 0.0; } );
  
 
   auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
   if (gait_flag==1){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);
gait_gen_->SetCombo(id_gait);
   }
    else if (gait_flag==2){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
gait_gen_->SetCombo(id_gait);
   }
     else if (gait_flag==3){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
gait_gen_->SetCombo(id_gait);
   }
      else if (gait_flag==4){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C7);
gait_gen_->SetCombo(id_gait);
   }
      else if (gait_flag==5){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C8);
gait_gen_->SetCombo(id_gait);
   }
      else if (gait_flag==6){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C9);
gait_gen_->SetCombo(id_gait);
   }




    formulation_.params_.ee_phase_durations_.clear();
    std::cout<<"Phase durations"<<formulation_.params_.ee_phase_durations_.size()<<std::endl;
    
    for (int ee=0; ee<4; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(0.5, ee));
      formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));

    }


  ifopt::Problem nlp;
  
  for (auto c : formulation_.GetVariableSets(solution))
    nlp.AddVariableSet(c);

  for (auto c : formulation_.GetConstraints(solution))
    nlp.AddConstraintSet(c);

  for (auto c : formulation_.GetCosts())
    nlp.AddCostSet(c);
     std::cout<<"size"<<formulation_.params_.GetEECount()<<std::endl;

 for (int ee=0; ee<4; ++ee) {
    for (int i=0; i<formulation_.params_.ee_phase_durations_.at(ee).size(); ++i){
  std::cout<<"size"<<ee<<std::endl;
  std::cout<<"size"<<formulation_.params_.ee_phase_durations_.at(ee)[i]<<std::endl;
  }}

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);
   
  

    return formulation_;


}




// Main 
int main(int argc, char *argv[])
{   
    //Load the urdf and create the quadruped
      std::string modelFile = argv[1];
    
     doggo= new QUADRUPED(modelFile);
    createrobot(modelFile);
    
      if( argc != 2 )
      {
        std::cerr << "KinDynComputationsWithEigen usage: KinDynComputationsWithEigen ./path/to/modelName.urdf" << std::endl;
        return EXIT_FAILURE;
      }

    // Helper class to load the model from an external format
      iDynTree::ModelLoader mdlLoader;

      bool ok = mdlLoader.loadModelFromFile(modelFile);

      if( !ok )
      {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return EXIT_FAILURE;
      }
   
    // Create a KinDynComputations class from the model
     iDynTree::KinDynComputations kinDynComp;
     ok = kinDynComp.loadRobotModel(mdlLoader.model());

     if( !ok )
     {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
     }
    
     const iDynTree::Model & model = kinDynComp.model();

    // Start node
     ros::init(argc, argv, "ros_control_node");
     ros::NodeHandle n;

    // Gazebo node 
     gazebo::transport::NodePtr node(new gazebo::transport::Node());
     node->Init();


    // Rate
      ros::Rate loop_rate(1000);
      gazebo::client::setup(argc,argv);


    // Ros  Subscribers
      ros::Subscriber joint_state_sub = n.subscribe("/dogbot/joint_states", 1, jointStateCallback);
      ros::Subscriber model_state_sub = n.subscribe("/gazebo/model_states", 1, modelStateCallback);
      ros::Subscriber object_state_sub = n.subscribe("/gazebo/link_states", 1, objectStateCallback);
    // Ros publisher
      front_left_knee_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_left_knee_position_controller/command", 1);
      front_right_knee_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_right_knee_position_controller/command", 1);
      back_left_knee_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_left_knee_position_controller/command", 1);
      back_right_knee_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_right_knee_position_controller/command", 1);
      front_left_pitch_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_left_pitch_position_controller/command", 1);
      front_right_pitch_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_right_pitch_position_controller/command", 1);
      back_left_pitch_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_left_pitch_position_controller/command", 1);
      back_right_pitch_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_right_pitch_position_controller/command", 1);
      front_left_roll_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_left_roll_position_controller/command", 1);
      front_right_roll_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/front_right_roll_position_controller/command", 1);
      back_left_roll_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_left_roll_position_controller/command", 1);
      back_right_roll_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/back_right_roll_position_controller/command", 1);
      shoulder_adduction_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint1_position_controller/command", 1);
      shoulder_flexion_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint2_position_controller/command", 1);
      humerus_rotation_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint3_position_controller/command", 1);
      elbow_flexion_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint4_position_controller/command", 1);
      wrist_rotation_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint5_position_controller/command", 1);
      wrist_flexion_effort_pub = n.advertise<std_msgs::Float64>("/dogbot/joint6_position_controller/command", 1);

    // Ros services
     ros::ServiceClient set_model_configuration_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
     ros::ServiceClient set_model_state_srv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
     ros::ServiceClient resetGazeboSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
     ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
     ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
     ros::ServiceClient obj_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
      des_pose_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
     ros::ServiceClient obj_get_dyn_srv = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
     _eebl_sub = n.subscribe("/dogbot/back_left_contactsensor_state",1, eebl_cb);
     _eefl_sub = n.subscribe("/dogbot/front_left_contactsensor_state",1, eefl_cb);
     _eebr_sub = n.subscribe("/dogbot/back_right_contactsensor_state",1, eebr_cb);
     _eefr_sub = n.subscribe("/dogbot/front_right_contactsensor_state",1, eefr_cb);
    // Gazebo publisher in case the qp problem takes too long for the control loop
      pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
      pub->WaitForConnection();

     // Set multi-step to requested iterations
       stepper.set_step(1);
  

    

    int dof= doggo->getDoFsnumber();
    std::cout<<"dof"<<dof<<std::endl;

    double massa= doggo->getMass();
    std::cout<<"massa"<<massa<<std::endl;
    
    gazebo_msgs::SetLinkState obj_init_state;
    obj_init_state.request.link_state.link_name = "cube::link_cube";
    obj_init_state.request.link_state.twist.linear.x = 0.0;
    obj_init_state.request.link_state.twist.linear.y = 0.0;
    obj_init_state.request.link_state.twist.linear.z = 0.0;
    obj_init_state.request.link_state.twist.angular.x = 0.0;
    obj_init_state.request.link_state.twist.angular.y = 0.0;
    obj_init_state.request.link_state.twist.angular.z = 0.0;
    obj_init_state.request.link_state.reference_frame = "world";
    obj_init_state.request.link_state.pose.position.x = -0.00181941; // 0.564973;
    obj_init_state.request.link_state.pose.position.y = -0.693907; // 0.372973
    obj_init_state.request.link_state.pose.position.z = 0.786604185104;//0.6399+0.008; // 0.766604185104-0.68136=0.08524
    obj_init_state.request.link_state.pose.orientation.x = 0.0;
    obj_init_state.request.link_state.pose.orientation.y = 0.0;
    obj_init_state.request.link_state.pose.orientation.z = 0.0;
    if(obj_set_state_srv.call(obj_init_state))
        ROS_INFO("Object state set.");
    else
        ROS_INFO("Failed to set object state.");


    // Start the robot in position (stand up) 
     gazebo_msgs::SetModelConfiguration robot_init_config;
      robot_init_config.request.model_name = "dogbot";
      robot_init_config.request.urdf_param_name = "robot_description";
      robot_init_config.request.joint_names.push_back("shoulder_adduction_joint");
      robot_init_config.request.joint_names.push_back("shoulder_flexion_joint");
      robot_init_config.request.joint_names.push_back("humerus_rotation_joint");
      robot_init_config.request.joint_names.push_back("elbow_flexion_joint");
      robot_init_config.request.joint_names.push_back("wrist_rotation_joint");
      robot_init_config.request.joint_names.push_back("wrist_flexion_joint");
      robot_init_config.request.joint_names.push_back("back_left_roll_joint");
      robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_left_knee_joint");
      robot_init_config.request.joint_names.push_back("back_right_roll_joint");
      robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_right_knee_joint");
      robot_init_config.request.joint_names.push_back("front_left_roll_joint");
      robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_left_knee_joint");
      robot_init_config.request.joint_names.push_back("front_right_roll_joint");
      robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_right_knee_joint");
      robot_init_config.request.joint_positions.push_back(-0);
      robot_init_config.request.joint_positions.push_back(-0.4);
      robot_init_config.request.joint_positions.push_back(-0);
      robot_init_config.request.joint_positions.push_back(0.4);
      robot_init_config.request.joint_positions.push_back(0);
      robot_init_config.request.joint_positions.push_back(0);
      robot_init_config.request.joint_positions.push_back(0.0037);
      robot_init_config.request.joint_positions.push_back(-0.8426);
      robot_init_config.request.joint_positions.push_back(-1.6216);
      robot_init_config.request.joint_positions.push_back(0.0039);
      robot_init_config.request.joint_positions.push_back(0.8421);
      robot_init_config.request.joint_positions.push_back(1.6165);
      robot_init_config.request.joint_positions.push_back(-0.0032);
      robot_init_config.request.joint_positions.push_back(-0.847);
      robot_init_config.request.joint_positions.push_back(-1.6302);
      robot_init_config.request.joint_positions.push_back(-0.0034);
      robot_init_config.request.joint_positions.push_back(0.8467);
      robot_init_config.request.joint_positions.push_back(1.6256);
      

      if(set_model_configuration_srv.call(robot_init_config))
        ROS_INFO("Robot configuration set.");
      else
        ROS_INFO("Failed to set robot configuration.");

      gazebo_msgs::SetModelState robot_init_state;
      robot_init_state.request.model_state.model_name = "dogbot";
      robot_init_state.request.model_state.reference_frame = "world";
      robot_init_state.request.model_state.pose.position.x=-0.00182;
      robot_init_state.request.model_state.pose.position.y=-0.0106414;
      robot_init_state.request.model_state.pose.position.z=0.426005;
      robot_init_state.request.model_state.pose.orientation.x=0.0;
      robot_init_state.request.model_state.pose.orientation.y=0.0;
      robot_init_state.request.model_state.pose.orientation.z=0.0;
      robot_init_state.request.model_state.pose.orientation.w=0.0;
      if(set_model_state_srv.call(robot_init_state))
        ROS_INFO("Robot state set.");
      else
        ROS_INFO("Failed to set robot state.");



    gazebo_msgs::GetLinkProperties obj_dyn_prop;
    obj_dyn_prop.request.link_name = "cube::link_cube";
    if(obj_get_dyn_srv.call(obj_dyn_prop))
    {
        ROS_INFO("Object dynamic properties retrieved.");
        obj_mass = obj_dyn_prop.response.mass;
        obj_inertia << obj_dyn_prop.response.ixx,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.iyy,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.izz;
        std::cout << "Object mass: " << obj_mass << std::endl;
        std::cout << "Object inertia: " << std::endl << obj_inertia << std::endl;
    }
    else
        ROS_INFO("Failed to get object dynamic properties.");


    gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "des_cube::des_cube";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = 0.522153;
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");

    std_srvs::Empty pauseSrv;

     while(!joint_state_available || !base_state_available )
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();
    }

   pauseGazebo.call(pauseSrv);

    // Create object
    double frictionCoeff = 0.6;
    std::vector<KDL::Frame> contacts(4);
    contacts.at(0).p = KDL::Vector(-0.03, 0.03,-0.03);
    contacts.at(1).p = KDL::Vector(0.03, 0.03,-0.03);
    contacts.at(2).p = KDL::Vector(0.03,-0.03,-0.03);
    contacts.at(3).p = KDL::Vector(-0.03,-0.03,-0.03);
    KDLObject obj(obj_mass, obj_inertia, contacts, frictionCoeff);
    std::cout << toEigenKDL(obj.getFrame().M) << std::endl;
    KDL::Wrench Fb(obj.getFrame().M.Inverse()*KDL::Vector(0,0,9.81*obj_mass), KDL::Vector(0,0,0));
    obj.computeContactForces(Fb);

    obj.setFrame(toKDL(obj_pos));
    doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);
    // Add object
    doggo->addObj(obj);
    // Update robot
     gravity<<0,0,-9.8;
     doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

    // control vector
      tau.resize(18);
      tau= Eigen::VectorXd::Zero(18);
      
      controller_= new QUADRUPEDController(*doggo);
      Eigen::Matrix<double,6,1>  obj1= doggo->getOBJpos();
    // Initial position
      Eigen::Matrix<double,6,1>init_position= doggo->getCOMpos();
    
    // Desired position
      Eigen::Matrix<double,6,1> end_position;
      end_position<<-0.01, -0.06, 0.4,0,0,0;
   
   
    // Initial velocity
      Eigen::Matrix<double,6,1> init_vel= doggo->getCOMvel();

    // Desired velocity
      Eigen::Matrix<double,6,1> end_vel= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
      Eigen::Matrix<double,6,1> init_acc= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
      Eigen::Matrix<double,6,1> end_acc= Eigen::MatrixXd::Zero(6,1); 

    // Trajectory
    
    

       // Set Gain Matrix
    Kcom=2000*Eigen::MatrixXd::Identity(6,6);
    Dcom=50*Eigen::MatrixXd::Identity(6,6);
    

    // initial simulation time 

     ROS_INFO_STREAM_ONCE("Starting control loop ...");
    int obj_phase=-1;
    int obj_phase2=-1;
 while(ros::ok)
 {   
    // initial simulation time
   formulation=computetrajecotry(1);

    begin3 = ros::Time::now();
     ROS_INFO_STREAM_ONCE("Starting control loop ...");
   
   double ti=0.0, tf=0.13, t = 0.0;

  Eigen::Matrix<double,6,1>init_positionsw;
  init_positionsw << obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];
    
    // Desired position
      Eigen::Matrix<double,6,1> deltaobj;
    
     Eigen::Vector3d dogang=doggo->getCOMpos().block(3,0,3,1);

        deltaobj<< 0.6281*sin(dogang[2]), -0.6281*cos(dogang[2]), 0.40229,  0, 0, 0;

     
     
      Eigen::Matrix<double,6,1> end_positionsw;
      end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(1)[0]).p()+deltaobj.block(0,0,3,1),
                      deltaobj.block(3,0,3,1);
    
   
    // Initial velocity
      Eigen::Matrix<double,6,1> init_velsw;
      init_velsw<<obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
      Eigen::Matrix<double,6,1> end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
      Eigen::Matrix<double,6,1> init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
      Eigen::Matrix<double,6,1> end_accsw= Eigen::MatrixXd::Zero(6,1); 
     TrajPlanner plannersw(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);

      Eigen::Matrix<double,6,1>init_positionswang;
      init_positionswang<< doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      Eigen::Matrix<double,6,1> end_positionswang;
      end_positionswang<< 0,0 , dogang[2]-0.025, 0, 0, 0;
    
   
    // Initial velocity
      Eigen::Matrix<double,6,1> init_velswang;
      init_velswang<<  doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
      Eigen::Matrix<double,6,1> end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
      Eigen::Matrix<double,6,1> init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
      Eigen::Matrix<double,6,1> end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajsw = plannersw.getTraj();

     trajswang = plannerswang.getTraj();
 begin32 = ros::Time::now();
   stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(1)[0]);




    init_positionsw << obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];
    
    // Desired position
    end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0]).p()+deltaobj.block(0,0,3,1),
                    solution.base_angular_->GetPoint(formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0]).p()+deltaobj.block(3,0,3,1);
    
   
    // Initial velocity
    init_velsw<< obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
    end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
    init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
    end_accsw= Eigen::MatrixXd::Zero(6,1); 
    TrajPlanner plannersw4(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);

    init_positionswang<<  doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      end_positionswang<< 0,0 , dogang[2]-0.025, 0, 0, 0;
    
   
    // Initial velocity
      init_velswang<<  doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
    end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
     init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
     end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang4(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajswang = plannerswang4.getTraj();
     
    begin32 = ros::Time::now();

     trajsw = plannersw4.getTraj();

   flag=0;
   swing_phase( loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);
    if (flag==1)
  {  
    flag=0;
    swing_phasebr(loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
  else if(flag==2)
  {  
    flag=0;
    swing_phasefl(loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
  


   tf=0.09;

    init_positionsw<< obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];

    // Desired position
    end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(0)[0]).p()+deltaobj.block(0,0,3,1),
                    deltaobj.block(3,0,3,1);
    


    // Initial velocity
    init_velsw<< obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
    end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
    init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
    end_accsw= Eigen::MatrixXd::Zero(6,1); 
    TrajPlanner plannersw1(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);


      init_positionswang<<  doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      end_positionswang<<  0,0 , dogang[2]-0.025, 0, 0, 0;
    
   
    // Initial velocity
      init_velswang<<  doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
    end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
     init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
     end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang1(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajswang = plannerswang1.getTraj();

     trajsw = plannersw1.getTraj();


    begin32 = ros::Time::now();
   stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(0)[0]);  

   formulation=computetrajecotry(2);

   begin3 = ros::Time::now();

   tf=0.12;
    init_positionsw<< obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];
    
    // Desired position
    end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(0)[0]).p()+deltaobj.block(0,0,3,1),
                    deltaobj.block(3,0,3,1);
    
   
    // Initial velocity
    init_velsw<< obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
    end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
    init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
    end_accsw= Eigen::MatrixXd::Zero(6,1); 
    TrajPlanner plannersw2(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);

    init_positionswang<< doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      end_positionswang<< 0,0 , dogang[2]-0.025, 0, 0, 0;
    
   
    // Initial velocity
      init_velswang<<  doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
    end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
     init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
     end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang2(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajswang = plannerswang2.getTraj();
     
    begin32 = ros::Time::now();

     trajsw = plannersw2.getTraj();

   stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(0)[0]);

   ///////////////

   tf=0.12;

    init_positionsw<< obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];
    
    // Desired position
    end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1]).p()+deltaobj.block(0,0,3,1),
                    deltaobj.block(3,0,3,1);
    
   
    // Initial velocity
    init_velsw<< obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
    end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
    init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
    end_accsw= Eigen::MatrixXd::Zero(6,1); 
    TrajPlanner plannersw5(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);

    init_positionswang<<  doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      end_positionswang<< 0,0 , dogang[2]-0.025, 0, 0, 0;

    // Initial velocity
      init_velswang<<doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
    end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
     init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
     end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang5(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajswang = plannerswang5.getTraj();
     
    begin32 = ros::Time::now();

     trajsw = plannersw5.getTraj();


   flag=0;
   swing_phase2( loop_rate, formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
    if (flag==1)
  { flag=0;
    swing_phasefr(loop_rate, formulation.params_.ee_phase_durations_.at(0)[1]+formulation.params_.ee_phase_durations_.at(0)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
  else if(flag==2)
  { flag=0;
    swing_phasebl(loop_rate, formulation.params_.ee_phase_durations_.at(0)[1]+formulation.params_.ee_phase_durations_.at(0)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
   
   
   
    tf=0.09;

    init_positionsw<< obj_pos[0],obj_pos[1],obj_pos[2], obj_pos[3],obj_pos[4],obj_pos[5];
    
    // Desired position
    end_positionsw<< solution.base_linear_->GetPoint(formulation.params_.ee_phase_durations_.at(1)[0]).p()+deltaobj.block(0,0,3,1),
                    deltaobj.block(3,0,3,1);
    
   
    // Initial velocity
    init_velsw<< obj_vel[0],obj_vel[1],obj_vel[2], obj_vel[3],obj_vel[4],obj_vel[5];

    // Desired velocity
    end_velsw= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
    init_accsw= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
    end_accsw= Eigen::MatrixXd::Zero(6,1); 
    TrajPlanner plannersw3(ti, tf, init_positionsw, end_positionsw, init_velsw, end_velsw, init_accsw, end_accsw);

    init_positionswang<< doggo->getOBJpos().block(3,0,3,1), 0, 0, 0;
    
    // Desired position
      end_positionswang<< 0,0 , dogang[2]-0.025, 0, 0, 0;
    
   
    // Initial velocity
      init_velswang<<  doggo->getOBJvel().block(3,0,3,1), 0, 0, 0;

    // Desired velocity
    end_velswang= Eigen::MatrixXd::Zero(6,1);

    // Initial acceleration
     init_accswang= Eigen::MatrixXd::Zero(6,1); 

    // Desired acceleration
     end_accswang= Eigen::MatrixXd::Zero(6,1);

     TrajPlanner plannerswang3(ti, tf, init_positionswang, end_positionswang, init_velswang, end_velswang, init_accswang, end_accswang);

     trajswang = plannerswang3.getTraj();
     
    begin32 = ros::Time::now();

     trajsw = plannersw3.getTraj();
    
    begin32 = ros::Time::now();
   stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(1)[0]);
        if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");  
}

/*formulation=computetrajecotry(3);

 begin3 = ros::Time::now();

stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(2)[0]);
flag=0;
swing_phasefl(loop_rate, formulation.params_.ee_phase_durations_.at(2)[1]+formulation.params_.ee_phase_durations_.at(2)[0] , formulation.params_.ee_phase_durations_.at(2)[0]);

stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(0)[0]);

formulation=computetrajecotry(4);

 begin3 = ros::Time::now();

stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(0)[0]);
flag=0;
swing_phasebl(loop_rate, formulation.params_.ee_phase_durations_.at(0)[1]+formulation.params_.ee_phase_durations_.at(0)[0] , formulation.params_.ee_phase_durations_.at(0)[0]);

stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(3)[0]);


formulation=computetrajecotry(5);

 begin3 = ros::Time::now();

stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(3)[0]);
flag=0;
swing_phasefr(loop_rate, formulation.params_.ee_phase_durations_.at(3)[1]+formulation.params_.ee_phase_durations_.at(3)[0] , formulation.params_.ee_phase_durations_.at(3)[0]);

stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(1)[0]);


formulation=computetrajecotry(6);

 begin3 = ros::Time::now();

stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(1)[0]);
flag=0;
swing_phasebr(loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);

stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(2)[0]);*/

  /* while ((ros::Time::now()-begin3).toSec() < tf)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo.update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         t = (ros::Time::now()-begin3).toSec();
         int idx=round( t*1000);
      std::cout<<"tau"<<tau<<std::endl;
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         toEigen(composdes)<<traj.pos(0,idx),traj.pos(1,idx),traj.pos(2,idx),0,0,0;
         toEigen(comveldes)<<traj.vel(0,idx),  traj.vel(1,idx), traj.vel(2,idx), 0,0,0;
         toEigen(comaccdes) <<traj.acc(0,idx), traj.acc(1,idx), traj.acc(2,idx), 0,0,0;
         
        std::cout<<"posdes"<<toEigen(composdes)<<std::endl;
        std::cout<<"veldes"<<toEigen(comveldes)<<std::endl;
        std::cout<<"accdes"<<toEigen(comaccdes)<<std::endl;
        
        std::cout<<"accdessw"<<trajsw.acc.block(0,idx,3,1)<<std::endl;
         std::cout<<"veldessw"<<trajsw.vel.block(0,idx,3,1)<<std::endl;
         std::cout<<"posdessw"<<trajsw.pos.block(0,idx,3,1)<<std::endl;

        Eigen::Matrix<double,6,1>  obj= doggo.getOBJpos();
        Eigen::Matrix<double,6,1>  objvel= doggo.getOBJvel();

        Eigen::MatrixXd Kp;
        Kp=250*Eigen::MatrixXd::Identity(3,3);
 
        if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");  
       Eigen::MatrixXd Kd;
        Kd=50*Eigen::MatrixXd::Identity(3,3);

        Eigen::Matrix<double,3,1> accdessw=trajsw.acc.block(0,idx,3,1)+Kd*(trajsw.vel.block(0,idx,3,1)-objvel.block(0,0,3,1))+Kp*(trajsw.pos.block(0,idx,3,1)-obj.block(0,0,3,1));
      // Compute control torque
        tau = controller_.Cntr(composdes, comveldes, comaccdes,
                                Kcom, Dcom, trajsw.acc.block(0,idx,3,1));

       std::cout<<"tau"<<tau<<std::endl;

      // Set command message

     
       front_left_knee_tau_msg.data=tau(6);
       front_right_knee_tau_msg.data=tau(12);
       back_left_knee_tau_msg.data=tau(10);
       back_right_knee_tau_msg.data=tau(8);
       front_left_pitch_tau_msg.data=tau(5);
       front_right_pitch_tau_msg.data=tau(11);
       back_left_pitch_tau_msg.data=tau(9);
       back_right_pitch_tau_msg.data=tau(7);
       front_left_roll_tau_msg.data=tau(4);
       front_right_roll_tau_msg.data=tau(1);
       back_left_roll_tau_msg.data=tau(2);
       back_right_roll_tau_msg.data=tau(3);
       shoulder_adduction_tau_msg.data=tau(0);
       shoulder_flexion_tau_msg.data=tau(13);
       humerus_rotation_tau_msg.data=tau(14);
       elbow_flexion_tau_msg.data=tau(15);
       wrist_rotation_tau_msg.data=tau(16);
       wrist_flexion_tau_msg.data=tau(17);

      // torques in right order

       //Sending command
      front_left_knee_effort_pub.publish(front_left_knee_tau_msg);
      front_right_knee_effort_pub.publish(front_right_knee_tau_msg);
      back_left_knee_effort_pub.publish(back_left_knee_tau_msg);
      back_right_knee_effort_pub.publish(back_right_knee_tau_msg);
      front_left_pitch_effort_pub.publish(front_left_pitch_tau_msg);
      front_right_pitch_effort_pub.publish(front_right_pitch_tau_msg);
      back_left_pitch_effort_pub.publish(back_left_pitch_tau_msg);
      back_right_pitch_effort_pub.publish(back_right_pitch_tau_msg);
      front_left_roll_effort_pub.publish(front_left_roll_tau_msg);
      front_right_roll_effort_pub.publish(front_right_roll_tau_msg);
      back_left_roll_effort_pub.publish(back_left_roll_tau_msg);
      back_right_roll_effort_pub.publish(back_right_roll_tau_msg);
      shoulder_adduction_effort_pub.publish(shoulder_adduction_tau_msg);
      shoulder_flexion_effort_pub.publish(shoulder_flexion_tau_msg);
      humerus_rotation_effort_pub.publish(humerus_rotation_tau_msg);
      elbow_flexion_effort_pub.publish(elbow_flexion_tau_msg);
      wrist_rotation_effort_pub.publish(wrist_rotation_tau_msg);
      wrist_flexion_effort_pub.publish(wrist_flexion_tau_msg);

    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
       // pub->Publish(stepper);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }*/


        if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");  

  
}