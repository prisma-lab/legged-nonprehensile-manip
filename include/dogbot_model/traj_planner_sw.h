#ifndef TRAJSWPlanner_H
#define TRAJSWPlanner_H

#include "Eigen/Dense"



struct trajectory_pointsw{

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


class TrajPlannerSw
{
    public:

     TrajPlannerSw(double ti, double tf, 
                        Eigen::Matrix<double,3,1> &initpos,
                        Eigen::Matrix<double,3,1> &endpos,
                        Eigen::Matrix<double,3,1> &initvel,
                        Eigen::Matrix<double,3,1> &endvel,
                        Eigen::Matrix<double,3,1> &initacc,
                        Eigen::Matrix<double,3,1> &endacc);

    
     trajectory_pointsw getTraj();

    private:

     double ti, tf;

     Eigen::Matrix<double,3,1> initpos, endpos, initvel, endvel, initacc, endacc;

     trajectory_pointsw traj;
    
     void createTraj();
};

#endif