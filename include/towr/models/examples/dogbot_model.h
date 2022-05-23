#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot ANYmal.
 */
class DogbotKinematicModel : public KinematicModel {
public:
  DogbotKinematicModel (const double z_value, const double z_dev) : KinematicModel(4)
  {
    const double x_nominal_b_lf = 0.1853;
    const double y_nominal_b_lf = 0.2304;
    const double z_nominal_b_lf = -z_value;

    const double x_nominal_b_rf = 0.1883;
    const double y_nominal_b_rf = 0.2295;
    const double z_nominal_b_rf = -z_value;

    const double x_nominal_b_lr = 0.1855;
    const double y_nominal_b_lr =  0.3446;
    const double z_nominal_b_lr = -z_value;

    const double x_nominal_b_rr = 0.1881;
    const double y_nominal_b_rr = 0.3455;
    const double z_nominal_b_rr = -z_value;


   /* const double base_delta_x = 0.0;
    const double base_delta_y = 0.0212;
    const double base_delta_z = 0.03;*/

   /* const double x_nominal_b_lf = 0.18656;
    const double y_nominal_b_lf = 0.302086;
    const double z_nominal_b_lf = -0.402152;

    const double x_nominal_b_rf = 0.186998;
    const double y_nominal_b_rf = 0.3011;
    const double z_nominal_b_rf = -0.402152;

    const double x_nominal_b_lr = 0.186756;
    const double y_nominal_b_lr = 0.273143;
    const double z_nominal_b_lr = -0.402152;

    const double x_nominal_b_rr = 0.18695;
    const double y_nominal_b_rr = 0.273887;
    const double z_nominal_b_rr = -0.402152;*/

    const double base_delta_x = 0.0;
    const double base_delta_y = 0.0;
    const double base_delta_z = 0.0;


    nominal_stance_.at(LF) <<  -x_nominal_b_lf,   -y_nominal_b_lf-base_delta_y, z_nominal_b_lf;//+base_delta_z;
    nominal_stance_.at(RF) <<  x_nominal_b_rf,   -y_nominal_b_rf-base_delta_y, z_nominal_b_rf;//+base_delta_z;
    nominal_stance_.at(LH) << -x_nominal_b_lr,   y_nominal_b_lr-base_delta_y, z_nominal_b_lr;//+base_delta_z;
    nominal_stance_.at(RH) << x_nominal_b_rr,  y_nominal_b_rr-base_delta_y, z_nominal_b_rr;//+base_delta_z;

    max_dev_from_nominal_ << 0.1, 0.15, z_dev;
  }
};

/**
 * @brief The Dynamics of the quadruped robot ANYmal.
 */
class DogbotDynamicModel : public SingleRigidBodyDynamics {
public:
  DogbotDynamicModel( const double Ixx_value, const double Iyy_value, const double Izz_value, const double Ixy_value, const double Ixz_value,  const double Iyz_value)
  : SingleRigidBodyDynamics( 24.261,
                             Ixx_value, Iyy_value, Izz_value, Ixy_value, Ixz_value, Iyz_value, 
                             
// 2.66983 , 0.72869,  3.05527,-8.85613e-05 ,4.20284e-05 ,0.187237  ,
                    4) {}
};

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_ */
