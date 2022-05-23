#ifndef TRAJPlanner_H
#define TRAJPlanner_H

#include "Eigen/Dense"



struct trajectory_point{

 void resize(int size)
 {
     pos.resize(3,size);
     vel.resize(3,size);
     acc.resize(3,size);
 }
  Eigen::MatrixXd pos;
  Eigen::MatrixXd vel;
  Eigen::MatrixXd acc;
};


class TrajPlanner
{
    public:

     TrajPlanner(double ti, double tf, 
                        Eigen::Matrix<double,6,1> &initpos,
                        Eigen::Matrix<double,6,1> &endpos,
                        Eigen::Matrix<double,6,1> &initvel,
                        Eigen::Matrix<double,6,1> &endvel,
                        Eigen::Matrix<double,6,1> &initacc,
                        Eigen::Matrix<double,6,1> &endacc);

    
     trajectory_point getTraj();

    private:

     double ti, tf;

     Eigen::Matrix<double,6,1> initpos, endpos, initvel, endvel, initacc, endacc;

     trajectory_point traj;
    
     void createTraj();
};

#endif