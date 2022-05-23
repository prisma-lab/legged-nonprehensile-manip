 #ifndef KDL_CONVERSIONS_HPP
 #define KDL_CONVERSIONS_HPP
 /*
 * expressiongraph library
 * 
 * Copyright 2014 Erwin Aertbelien - KU Leuven - Dep. of Mechanical Engineering
 *
 * Licensed under the EUPL, Version 1.1 only (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * http://ec.europa.eu/idabc/eupl 
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence.
 */
  
  
  
 #include <Eigen/Dense>
 #include <kdl/frames.hpp>
 namespace KDL {
     inline Vector toKDLVector( const Eigen::Vector3d& arg) {
         Vector tmp( arg(0), arg(1), arg(2) );
         return tmp;
     }
  
  
    inline  Vector toKDLVector( const Eigen::VectorXd& arg) {
         assert( arg.rows()==3 );
         Vector tmp( arg(0), arg(1), arg(2) );
         return tmp;
     }
  
     inline Eigen::Vector3d toEigen( const KDL::Vector& arg ) {
         Eigen::Vector3d tmp;
         tmp(0)=arg.x();
         tmp(1)=arg.y();
         tmp(2)=arg.z();
         return tmp;
     }
  
  
     inline KDL::Twist toKDLTwist( const Eigen::Matrix<double,6,1>& arg) {
         Twist tmp( Vector(arg(0), arg(1), arg(2)), Vector( arg(3),arg(4),arg(5)) );
         return tmp;
     }
  
  
     inline KDL::Twist toKDLTwist( const Eigen::VectorXd& arg) {
         assert( arg.rows()==6 );
         Twist tmp( Vector(arg(0), arg(1), arg(2)), Vector( arg(3),arg(4),arg(5)) );
         return tmp;
     }
  
     inline Eigen::Matrix<double,6,1> toEigen( const KDL::Twist& arg ) {
         Eigen::Matrix<double,6,1> tmp;
         tmp(0)=arg.vel.x();
         tmp(1)=arg.vel.y();
         tmp(2)=arg.vel.z();
         tmp(3)=arg.rot.x();
         tmp(4)=arg.rot.y();
         tmp(5)=arg.rot.z();
         return tmp;
     }
     
     inline KDL::Wrench toKDLWrench( const Eigen::Matrix<double,6,1>& arg) {
         Wrench tmp( Vector(arg(0), arg(1), arg(2)), Vector( arg(3),arg(4),arg(5)) );
         return tmp;
     }
  
     inline KDL::Wrench toKDLWrench( const Eigen::VectorXd& arg) {
         assert( arg.rows()==6 );
         Wrench tmp( Vector(arg(0), arg(1), arg(2)), Vector( arg(3),arg(4),arg(5)) );
         return tmp;
     }
  
     inline Eigen::Matrix<double,6,1> toEigen( const KDL::Wrench& arg ) {
         Eigen::Matrix<double,6,1> tmp;
         tmp(0)=arg.force.x();
         tmp(1)=arg.force.y();
         tmp(2)=arg.force.z();
         tmp(3)=arg.torque.x();
         tmp(4)=arg.torque.y();
         tmp(5)=arg.torque.z();
         return tmp;
     }
  
     inline KDL::Rotation toKDLRotation(const Eigen::Matrix<double,3,3>& arg) {
         KDL::Rotation R( 
                         arg(0,0), arg(0,1), arg(0,2),
                         arg(1,0), arg(1,1), arg(1,2),
                         arg(2,0), arg(2,1), arg(2,2) 
         );
         return R;                  
     }
     
     inline KDL::Rotation toKDLRotation(const Eigen::MatrixXd& arg) {
         assert(arg.rows()==3); 
         assert(arg.cols()==3); 
         KDL::Rotation R( 
                         arg(0,0), arg(0,1), arg(0,2),
                         arg(1,0), arg(1,1), arg(1,2),
                         arg(2,0), arg(2,1), arg(2,2) 
         );
         return R; 
     }
    
     inline Eigen::Matrix<double,3,3> toEigen(const KDL::Rotation& arg) {
         Eigen::Matrix<double,3,3> m;
         m.col(0) = toEigen( arg.UnitX() );
         m.col(1) = toEigen( arg.UnitY() );
         m.col(2) = toEigen( arg.UnitZ() );
         return m;
     }
  
     inline KDL::Rotation toKDLRotation(const Eigen::Quaternion<double>& arg) {
         return toKDLRotation( arg.toRotationMatrix() );
     }
    
     inline Eigen::Quaternion<double> toEigenQuaternion(const KDL::Rotation& arg) {
         return Eigen::Quaternion<double>( toEigen(arg) );
     }
  
 };
 #endif

