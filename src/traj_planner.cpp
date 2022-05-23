#include "dogbot_model/traj_planner.h"



TrajPlanner::TrajPlanner(double ti_, double tf_, 
                        Eigen::Matrix<double,6,1> &init_pos,
                        Eigen::Matrix<double,6,1> &end_pos,
                        Eigen::Matrix<double,6,1> &init_vel,
                        Eigen::Matrix<double,6,1> &end_vel,
                        Eigen::Matrix<double,6,1> &init_acc,
                        Eigen::Matrix<double,6,1> &end_acc)
{
    // Initial time
      ti=ti_;

    //Trajectory Duration
      tf=tf_;

    //Initial position
      initpos=init_pos;
  
    //Final position
     endpos=end_pos;
     
    //Initial velocity
      initvel=init_vel;

    //Final velocity
      endvel=end_vel;

    //Initial acceleration
      initacc=init_acc;

    //Final acceleration
      endacc=end_acc;     

      createTraj();

}


void TrajPlanner::createTraj()
{   
     // Compute trajectory
     Eigen::Matrix<double,6,6> Ax;

      Ax<< pow(ti,5), pow(ti,4), pow(ti,3), pow(ti,2), ti, 1,
           5*pow(ti,4), 4*pow(ti,3), 3*pow(ti,2), 2*ti, 1, 0,
           20*pow(ti,3), 12*pow(ti,2), 6*ti, 2, 0, 0,
           pow(tf,5), pow(tf,4), pow(tf,3), pow(tf,2), tf, 1,
           5*pow(tf,4), 4*pow(tf,3), 3*pow(tf,2), 2*(tf), 1, 0,
           20*pow(tf,3), 12*pow(tf,2), 6*tf, 2, 0, 0;

     Eigen::Matrix<double,6,1> bx;
    
     bx<<initpos(0,0), initvel(0,0), initacc(0,0), endpos(0,0), endvel(0,0),endacc(0,0);
     

     // X coefficient
         Eigen::Matrix<double,6,1> coeffx=Ax.inverse()*bx;


     Eigen::Matrix<double,6,1> by;

     by<<initpos(1,0), initvel(1,0), initacc(1,0), endpos(1,0), endvel(1,0),endacc(1,0);
     
     // Y coefficient
         Eigen::Matrix<double,6,1> coeffy=Ax.inverse()*by;

     Eigen::Matrix<double,6,1> bz;

     bz<<initpos(2,0), initvel(2,0), initacc(2,0), endpos(2,0), endvel(2,0),endacc(2,0);
    

     // Z coefficient
         Eigen::Matrix<double,6,1> coeffz=Ax.inverse()*bz;


     // Trajectory duration
         double traj_duration= tf-ti;

     // Trajectory vector dimension
         double size= traj_duration*1000;
 
        traj.resize(size);

     for(int i=0; i<size; i++)
     {   double tn=0.001*i;
        
        traj.pos(0,i)=coeffx(5,0)+coeffx(4,0)*tn+coeffx(3,0)*pow(tn,2)+coeffx(2,0)*pow(tn,3)+coeffx(1,0)*pow(tn,4)+coeffx(0,0)*pow(tn,5);
        traj.pos(1,i)=coeffy(5,0)+coeffy(4,0)*tn+coeffy(3,0)*pow(tn,2)+coeffy(2,0)*pow(tn,3)+coeffy(1,0)*pow(tn,4)+coeffy(0,0)*pow(tn,5);
        traj.pos(2,i)=coeffz(5,0)+coeffz(4,0)*tn+coeffz(3,0)*pow(tn,2)+coeffz(2,0)*pow(tn,3)+coeffz(1,0)*pow(tn,4)+coeffz(0,0)*pow(tn,5);
        
        traj.vel(0,i)=coeffx(4,0)+2*coeffx(3,0)*pow(tn,1)+3*coeffx(2,0)*pow(tn,2)+4*coeffx(1,0)*pow(tn,3)+5*coeffx(0,0)*pow(tn,4);
        traj.vel(1,i)=coeffy(4,0)+2*coeffy(3,0)*pow(tn,1)+3*coeffy(2,0)*pow(tn,2)+4*coeffy(1,0)*pow(tn,3)+5*coeffy(0,0)*pow(tn,4);
        traj.vel(2,i)=coeffz(4,0)+2*coeffz(3,0)*pow(tn,1)+3*coeffz(2,0)*pow(tn,2)+4*coeffz(1,0)*pow(tn,3)+5*coeffz(0,0)*pow(tn,4);
        
        traj.acc(0,i)=2*coeffx(3,0)+6*coeffx(2,0)*pow(tn,1)+12*coeffx(1,0)*pow(tn,2)+20*coeffx(0,0)*pow(tn,3);
        traj.acc(1,i)=2*coeffy(3,0)+6*coeffy(2,0)*pow(tn,1)+12*coeffy(1,0)*pow(tn,2)+20*coeffy(0,0)*pow(tn,3);
        traj.acc(2,i)=2*coeffz(3,0)+6*coeffz(2,0)*pow(tn,1)+12*coeffz(1,0)*pow(tn,2)+20*coeffz(0,0)*pow(tn,3);
     }
 }

trajectory_point TrajPlanner::getTraj()
{
    return traj;

}
 


