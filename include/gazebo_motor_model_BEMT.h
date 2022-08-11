/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */



#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"
#include "Wind.pb.h"

#include "common.h"
#include <cmath>
#include <cstring>
#include <fstream>


typedef struct {
  Eigen::ArrayXd raduis, r_R, chord, c_R, beta;
  int station, blades;
  double modify_Pitch, scale_Raduis;
  std::vector<std::string> airfoil;
  double airfoil5_database [31][721][3];
  double airfoil10_database [31][721][3];
  double airfoil15_database [31][721][3];
  Eigen::ArrayXd chord_Mid, omega_Mid, sigma_Mid;


  Eigen::ArrayXd dT, dD, dL, dCT;
  double T, D, L, CT;
}Blade;

typedef struct {
  double rho = 1.225;
  double mu = 1.8e-5;
  double inflow_Angle;
  double V, Vrp_a, Vrp_p;
  Eigen::ArrayXd Vaz_perp, Vaz_tang, V_R;
  double Re_mid;
  double ui,vi,lambda;
}Flow;

typedef struct {
  double rps;
  double azimuth_Num;
  double alpha_Zero = -0.3;
  double a_0 = 2*M_PI ;
  int inflow_Type;
  int toggle_Visc; 
  int toggle_Vi; 
  int toggle_WIM;
}Oper;

// struct Flow {};
// struct Oper {};
// struct Rotor {};
// struct Wake {};
// struct Options {};

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
std::string wind_sub_topic_ = "/world_wind";

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

/*
// Protobuf test
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
static const std::string kDefaultMotorTestSubTopic = "motors";
*/

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(kDefaultCommandSubTopic),
        motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic),
        motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),
        motor_number_(0),
        motor_Failure_Number_(0),
        turning_direction_(turning_direction::CW),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        moment_constant_(kDefaultMomentConstant),
        motor_constant_(kDefaultMotorConstant),
        //motor_test_sub_topic_(kDefaultMotorTestSubTopic),
        ref_motor_rot_vel_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp),
        reversible_(false) {
  }

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();
  //void testProto(MotorSpeedPtr &msg);
 protected:
  virtual void UpdateForcesAndMoments();
  /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
  /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
  virtual void UpdateMotorFail();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
  void BEMT(Blade* _blade,Flow* _flow,Oper* _oper);
  void UniformMomentumFF(Blade* _blade,Flow* _flow);


 private:
  std::string command_sub_topic_;
  std::string motor_failure_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  int motor_number_;
  int turning_direction_;

  int motor_Failure_Number_; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
  int tmp_motor_num; // A temporary variable used to print msg

  int screen_msg_flag = 1;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_rot_vel_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  bool reversible_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_pub_;
  transport::SubscriberPtr command_sub_;
  transport::SubscriberPtr motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */
  transport::SubscriberPtr wind_sub_;

  ignition::math::Vector3d wind_vel_;

  Blade blade_; Blade* BladePtr = &blade_;
  Flow flow_; Flow* FlowPtr = &flow_;
  Oper oper_; Oper* OperPtr = &oper_;


  void loadAirfoilDatabase(double airfoil_database[31][721][3], std::string filename){
    int num_Re = 31;
    int num_angle = 721;
    int num_coeff = 3;

    std::ifstream file_input(filename);

    if(!file_input.is_open()){
      std::cout << "couldn't open "<< filename <<std::endl;
    }
        
        for (int Re = 0; Re < num_Re ; Re++) 
        {
            for (int AoA = 0; AoA < num_angle; AoA++) 
            {
              for (int coeff = 0; coeff < num_coeff; coeff++) 
              {
                file_input >> airfoil_database[Re][AoA][coeff]; 
              }
            }
        }

    file_input.close();

  }

  double trapz(Eigen::ArrayXd X, Eigen::ArrayXd Y){ 
    
    int size = X.size();
    Eigen::ArrayXd Z;
    Z = (X.tail(size-1)-X.head(size-1)) * (Y.head(size-1)+Y.tail(size-1))/2.0;
    double outPut = Z.sum();
    return outPut;
    }

  Eigen::ArrayXd simps(Eigen::ArrayXd X, Eigen::ArrayXXd Y){ 
    int n = Y.cols();
    int sizeX = X.size();

    Eigen::ArrayXd h = X.tail(sizeX-1)-X.head(sizeX-1);
    Eigen::ArrayXd dx1 = h.head(h.size()-1);
    Eigen::ArrayXd dx2 = h.tail(h.size()-1);

    Eigen::ArrayXd Alpha2 = (dx1+dx2)/dx1/6.0;
    Eigen::ArrayXd a0 = Alpha2*(2*dx1-dx2);
    Eigen::ArrayXd a1 = Alpha2*(dx1+dx2)*(dx1+dx2)/dx2;
    Eigen::ArrayXd a2 = Alpha2*dx1/dx2*(2.0*dx2-dx1);

    Eigen::Map<Eigen::ArrayXd,0,Eigen::InnerStride<2>> a0_2(a0.data(), std::round(a0.size()/2.0));
    Eigen::Map<Eigen::ArrayXd,0,Eigen::InnerStride<2>> a1_2(a1.data(), std::round(a1.size()/2.0));
    Eigen::Map<Eigen::ArrayXd,0,Eigen::InnerStride<2>> a2_2(a2.data(), std::round(a2.size()/2.0));

    Eigen::ArrayXXd Y_1_m2 = Y.block(0,0,(Y.rows()-2.0),Y.cols());
    Eigen::ArrayXXd even_Y_1_m2 =  Eigen::ArrayXXd::Map(Y_1_m2.data(),std::round((Y_1_m2.rows())/2.0), Y_1_m2.cols(),Eigen::Stride<Eigen::Dynamic,2>(Y_1_m2.rows(),2));


    Eigen::ArrayXXd Y_2_m1 = Y.block(1,0,(Y.rows()-2.0),Y.cols());
    Eigen::ArrayXXd even_Y_2_m1 = Eigen::ArrayXXd::Map(Y_2_m1.data(),std::round((Y_2_m1.rows())/2.0), Y_2_m1.cols(),Eigen::Stride<Eigen::Dynamic,2>(Y_2_m1.rows(),2));


    Eigen::ArrayXXd Y_3_m = Y.block(2,0,(Y.rows()-2.0),Y.cols());
    Eigen::ArrayXXd even_Y_3_m = Eigen::ArrayXXd::Map(Y_3_m.data(),std::round((Y_3_m.rows())/2.0), Y_3_m.cols(),Eigen::Stride<Eigen::Dynamic,2>(Y_3_m.rows(),2));

    Eigen::ArrayXXd Z = a0_2.replicate(1,n)*even_Y_1_m2 + a1_2.replicate(1,n)*even_Y_2_m1 + a2_2.replicate(1,n)*even_Y_3_m;

    return Z.colwise().sum();
    }

  double* coeffLookup(int Re, double alpha, double  airfoil_database[31][721][3]){
    int i = (int) (Re / 10000);
    int j = (int) ((std::round(alpha*2.0)/2.0 + 180)*2.0);
    // std::cout << "alpha="<<std::round(alpha*2.0)/2.0 << "   i= "<< i<< "   j="<<j<<"    ";

    double* outPut = new double[3];
    outPut[0] = airfoil_database[i][j][0];
    outPut[1] = airfoil_database[i][j][1];
    outPut[2] = airfoil_database[i][j][2];
    
    return outPut;
    
  }

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  common::PID pid_;
  bool use_pid_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  std_msgs::msgs::Float turning_velocity_msg_;
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
  void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);  /*!< Callback for the motor_failure_sub_ subscriber */
  void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
/*
  // Protobuf test
  std::string motor_test_sub_topic_;
  transport::SubscriberPtr motor_sub_;
*/
};
}
