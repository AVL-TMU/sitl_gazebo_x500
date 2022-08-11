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


#include "gazebo_motor_model_BEMT.h"
#include <ignition/math.hh>

#include <filesystem>

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
  updateConnection_->~Connection();
  use_pid_ = false;
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  // FIXME: Commented out to prevent warnings about queue limit reached.
  // motor_velocity_pub_->Publish(turning_velocity_msg_);
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");

  // setup joint control pid to control joint
  if (_sdf->HasElement("joint_control_pid"))
  {
    sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
    double p = 0.1;
    if (pid->HasElement("p"))
      p = pid->Get<double>("p");
    double i = 0;
    if (pid->HasElement("i"))
      i = pid->Get<double>("i");
    double d = 0;
    if (pid->HasElement("d"))
      d = pid->Get<double>("d");
    double iMax = 0;
    if (pid->HasElement("iMax"))
      iMax = pid->Get<double>("iMax");
    double iMin = 0;
    if (pid->HasElement("iMin"))
      iMin = pid->Get<double>("iMin");
    double cmdMax = 3;
    if (pid->HasElement("cmdMax"))
      cmdMax = pid->Get<double>("cmdMax");
    double cmdMin = -3;
    if (pid->HasElement("cmdMin"))
      cmdMin = pid->Get<double>("cmdMin");
    pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    use_pid_ = true;
  }
  else
  {
    use_pid_ = false;
  }

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  if(_sdf->HasElement("reversible")) {
    reversible_ = _sdf->GetElement("reversible")->Get<bool>();
  }

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  /*
  std::cout << "Subscribing to: " << motor_test_sub_topic_ << std::endl;
  motor_sub_ = node_handle_->Subscribe<mav_msgs::msgs::MotorSpeed>("~/" + model_->GetName() + motor_test_sub_topic_, &GazeboMotorModel::testProto, this);
  */

  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboMotorModel::VelocityCallback, this);
  //std::cout << "[gazebo_motor_model]: Subscribe to gz topic: "<< motor_failure_sub_topic_ << std::endl;
  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &GazeboMotorModel::MotorFailureCallback, this);
  // FIXME: Commented out to prevent warnings about queue limit reached.
  //motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);
  wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &GazeboMotorModel::WindVelocityCallback, this);

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));

blade_.raduis.resize(20);
blade_.raduis << 0.0114, 0.0229, 0.0343, 0.0457, 0.0572, 0.0686, 0.0800, 0.0914, 0.1029, 0.1143, 0.1257, 0.1372, 0.1486, 0.1600, 0.1715, 0.1829, 0.1943, 0.2057, 0.2172, 0.2286; 

blade_.r_R.resize(20);
blade_.r_R << 0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1.0;

blade_.chord.resize(20);
blade_.chord << 0.0194,0.0199,0.0253,0.0332,0.0386,0.0420,0.0443,0.0438,0.0430,0.0416,0.0400,0.0386,0.0365,0.0343,0.0327,0.0305,0.0278,0.0248,0.0211,0.0091;

blade_.c_R.resize(20);
blade_.c_R << 0.0850,0.0869,0.1107,0.1454,0.1691,0.1838,0.1938,0.1917,0.1881,0.1819,0.1751,0.1688,0.1595,0.1500,0.1430,0.1334,0.1216,0.1086,0.0922,0.0398;

blade_.beta.resize(20);
blade_.beta << 1.29,1.29,15.763,22.586,21.375,19.727,17.023,15.476,13.973,12.472,11.9,10.937,10.536,9.683,9.268,8.885,8.336,7.352,7.123,7.526;

blade_.station = 20; blade_.blades = 2; blade_.scale_Raduis = 1; blade_.modify_Pitch = -1.5;

blade_.airfoil ={"airfoil5","airfoil5","airfoil5","airfoil5","airfoil5","airfoil10","airfoil10","airfoil10","airfoil10","airfoil10","airfoil10","airfoil10",
"airfoil15","airfoil15","airfoil15","airfoil15","airfoil15","airfoil15","airfoil15","airfoil15"};

std::string filepath_airfoil5 = "/home/aminys/PX4-Autopilot/Tools/sitl_gazebo/resources/airfoil_database/airfoil5_coeff_database.txt";
loadAirfoilDatabase(BladePtr->airfoil5_database,filepath_airfoil5);


std::string filepath_airfoil10 = "/home/aminys/PX4-Autopilot/Tools/sitl_gazebo/resources/airfoil_database/airfoil10_coeff_database.txt";
loadAirfoilDatabase(BladePtr->airfoil10_database,filepath_airfoil10);


std::string filepath_airfoil15 = "/home/aminys/PX4-Autopilot/Tools/sitl_gazebo/resources/airfoil_database/airfoil15_coeff_database.txt";
loadAirfoilDatabase(BladePtr->airfoil15_database,filepath_airfoil15);

}

// Protobuf test
/*
void GazeboMotorModel::testProto(MotorSpeedPtr &msg) {
  std::cout << "Received message" << std::endl;
  std::cout << msg->motor_speed_size()<< std::endl;
  for(int i; i < msg->motor_speed_size(); i++){
    std::cout << msg->motor_speed(i) <<" ";
  }
  std::cout << std::endl;
}
*/

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  UpdateMotorFail();
  Publish();
  // std::cout  << "BEMT MODEL ";
}

void GazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void GazeboMotorModel::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg) {
  motor_Failure_Number_ = fail_msg->data();
}

void GazeboMotorModel::UpdateForcesAndMoments() {
  motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;
  if(!reversible_) {
    // Not allowed to have negative thrust.
    force = std::abs(force);
  }

  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  //
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
  ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();
  ignition::math::Vector3d euler = T_W_I.Rot().Euler();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
#endif




  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;



  ignition::math::Vector3d velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  double vel = velocity_parallel_to_rotor_axis.Length();
  double scalar = 1 - vel / 25.0; // at 25 m/s the rotor will not produce any force anymore
  scalar = ignition::math::clamp(scalar, 0.0, 1.0);

  // FlowPtr->V = vel;

  FlowPtr->V = 5.0;
  FlowPtr->inflow_Angle = 10.0* M_PI/180.0;
  // OperPtr-> rps = real_motor_velocity ;
  OperPtr-> rps = 3000.0/60.0;
  OperPtr->azimuth_Num = 8;

  OperPtr->inflow_Type = 1;
  OperPtr->toggle_Visc = 1;  // 1 -> "on", 0 -> "off"
  OperPtr->toggle_Vi = 1;    // 1 -> "on", 0 -> "off"
  OperPtr->toggle_WIM = 0;    // 1 -> "on", 0 -> "off"

  BEMT(BladePtr,FlowPtr,OperPtr);










  // Apply a force to the link.
  link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force * scalar));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  ignition::math::Vector3d velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // Apply air_drag to link.
  link_->AddForce(air_drag);
  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
  ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * force * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  ignition::math::Vector3d rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

#if 0 //FIXME: disable PID for now, it does not play nice with the PX4 CI system.
  if (use_pid_)
  {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
    // gzerr << "rotor " << joint_->GetName() << " : " << rotorForce << "\n";
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 7
    // Not desirable to use SetVelocity for parts of a moving model
    // impact on rest of the dynamic system is non-physical.
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#elif GAZEBO_MAJOR_VERSION >= 6
    // Not ideal as the approach could result in unrealistic impulses, and
    // is only available in ODE
    joint_->SetParam("fmax", 0, 2.0);
    joint_->SetParam("vel", 0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif
  }
#else
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif /* if 0 */
}

void GazeboMotorModel::UpdateMotorFail() {
  if (motor_number_ == motor_Failure_Number_ - 1){
    // motor_constant_ = 0.0;
    joint_->SetVelocity(0,0);
    if (screen_msg_flag){
      std::cout << "Motor number [" << motor_Failure_Number_ <<"] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;

      screen_msg_flag = 0;
    }
  }else if (motor_Failure_Number_ == 0 && motor_number_ ==  tmp_motor_num - 1){
     if (!screen_msg_flag){
       //motor_constant_ = kDefaultMotorConstant;
       std::cout << "Motor number [" << tmp_motor_num <<"] running! [Motor thrust = (default)]" << std::endl;
       screen_msg_flag = 1;
     }
  }
}

void GazeboMotorModel::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}










void GazeboMotorModel::BEMT(Blade* _blade,Flow* _flow, Oper* _oper){


// Extract variables from blade structure
double V = _flow->V;                                // Flight speed in [m/s].
double AOA = _flow->inflow_Angle;                   // Vehicle Angle of Attack, Inflow angle to rotor plane [rad]
double station = _blade->station;                   // Number of stations
double B = _blade->blades;                          // Number of blades.
double R = _blade->raduis.coeff(station-1);           // Rotor radius.
Eigen::ArrayXd chord = _blade->c_R * R;             // Dimensional chord lengths [m] 
Eigen::ArrayXd y_Span = _blade->r_R * R;            // 
double omega = _oper->rps * 2.0 * M_PI;

// Azimuth position setup
double azm = _oper->azimuth_Num;
Eigen::ArrayXd azimuth;
if (azm == 2.0)
{
  azimuth << M_PI/2.0,3.0*M_PI/2.0;
}else{
  azimuth = Eigen::ArrayXd::LinSpaced(azm,0,2.0*M_PI*(1.0-1.0/azm));
}



// Mid-station setup
Eigen::ArrayXd span_Mid = (y_Span.head(station-1)+y_Span.tail(station-1))/2.0;
Eigen::ArrayXd beta_Mid = (_blade->beta.head(station-1)+_blade->beta.tail(station-1))/2.0;
Eigen::ArrayXd r_R_Mid = (_blade->r_R.head(station-1)+_blade->r_R.tail(station-1))/2.0;
_blade->chord_Mid = (chord.head(station-1)+chord.tail(station-1))/2.0;
std::vector<std::string> airfoil_Mid = _blade->airfoil;
airfoil_Mid.erase(airfoil_Mid.begin());
_blade->sigma_Mid = (B*_blade->chord_Mid)/(2*M_PI*span_Mid);

// Initial calculations
    //  0 [deg] is back of rotor plane. Spins CCW: blade advances first
_blade->omega_Mid = span_Mid*omega;

// Velocity component normal to rotor plane
_flow->Vrp_a = V * sin(AOA);
_flow->Vrp_p = V * cos(AOA);

//  Velocity component parallel to disk, wrt blade frame
_flow->Vaz_perp = _flow->Vrp_p * azimuth.sin();
_flow->Vaz_tang = _flow->Vrp_p * azimuth.cos();

// Advance ratios
double mu_x = _flow->Vrp_p/(omega*R);       // Rotor advance ratio parallel to disk (tangential).
double mu_z = _flow->Vrp_a/(omega*R);       // Rotor advance ratio normal to disk (axial).
double mu_freestream = V/(omega*R);  // Freestream advance ratio
double J = V/(_oper->rps*2*R);  // Propeller advance ratio.
    
// Global solidity. Excluding hub area
double global_sigma = (B*trapz(y_Span,chord))/((M_PI*(R*R))-(M_PI*(y_Span.coeff(0)*y_Span.coeff(0))));  //Sectional solidity ratio.


// Rotor inflow
//  Inflow Models
    if ((V != 0 && AOA != 90) && (_oper->inflow_Type != 1) && (_oper->inflow_Type != 5)){
        _oper->inflow_Type = 1; // Switch to FF model if case 1 or 2 is accidentally selected.
        std::cout << " ** warning:\n Inflow reverting to uniform momentum with FF inflow model, Vrp_p =/= 0. Rotor had edgewise velocity component";
    }
                                                                                        //  ??                 
    // if (strcmp(_oper->toggle_Visc,'off') && _oper->inflow_Type > 2 && (V == 90 || _flow->inflow_angle)){
    //     warning('Viscous effects toggle is off, reverting to uniform inflow model 2');
    //     _oper.inflow_type = 2;
    // } ???
    




    // Returns: vi and ui in LOCAL ROTOR reference frame.
    try{
        switch   (_oper->toggle_Vi){
        //  BEMT
            case 1:
                switch (_oper->inflow_Type){
                    case 1:{
                        UniformMomentumFF(_blade, _flow);
                        _flow->lambda = (_flow->Vrp_a+_flow->vi)/(omega*R);
                        _flow->ui = 0;
                        break;
                    }
                    case 2:
                        // Inflow_small_angles_no_swirl
                        _flow->ui =  0;
                        break;
                    case 3:
                        // Inflow_small_angles_no_swirl_coeff_lookup
                        _flow->ui =  0;
                        break;
                    case 4:
                        // Inflow_large_angles_and_swirl
                        // ui = (bsxfun(@times,omega_mid,ones(1,azm)))-(ksi.*omega.*R);
                        break;
                    case 5:
                        // Inflow_WIM_single_rotor; // LOCAL COORDINATES TO BLADE POSITION! % Get z (or inflow) component in BLADE coordinate system
                        break;
                }
        //  BET (No induced velocity)
            case 0:
                _flow->lambda  = _flow->Vrp_a/(omega*R);
                _flow->ui =  0;
        
    }
    }
    catch(const std::exception& e){ // Catch when the induced velocity is non-real
        std::cerr << e.what() << '\n';
        std::cout << "**warning  Error in inflow calculaiton: Reverting now to BET forumlation','on','backtrace','on','verbose";
        _flow->lambda  = _flow->Vrp_a/(omega*R);
    }

    
// // Blade velocities, sectional coefficients and adjacent wake effects

// //  Update to include wake effects on adjacent velocities at rotor plane.
// //  Included in freestream velocity components
//     //  if (_oper->toggle_WIM == 1){
//     //          Vrp_a = Vrp_a.*ones(stations-1,1) + wake.vi_WIM(rotor.WIM_rotor_counter) ;
//     //          Vrp_p = Vrp_p.*ones(stations-1,1) + wake.vi_WIM(rotor.WIM_rotor_counter) ;
//     //  }

// //  Inflow angle, (Pitch - inflow angle = Effective AoA)
//       double phi  = math::atan2((lambda*(omega*R)),Vaz_perp+omega_mid-ui);
// //  Resultant velocity at each element
//       double V_R  = math::sqrt((lambda*(omega.*R))*(lambda*(omega.*R))+(Vaz_perp+omega_mid-ui)*(Vaz_perp+omega_mid-ui));
// //  Reynolds number at each element
//       double Re_mid = _flow->rho*(V_R*chord_mid)/(_flow->mu);
// //  Mach number at each element, assuming perfect gas @ sea level
//       double Mach_mid = math::abs(V_R)/340.0;

// //   Determine section AoA and coefficients
// //     % beta_zero       =       find_zero_lift_angle_airfoil(Re_mid,airfoil_mid);% Zero lift angle of attack [rad] (Beta for prop 2D section) 
//      double alpha_0 = beta_mid - phi;   

// //  Coefficient gathering methods
// //      Database look-up option, viscous effects on
//     if (_oper->toggle_visc==1 && _options->toggle_precompute==1){

//         lookup_aero_coeff_database(_options,_options->airfoil_coeff_database,alpha_0,Re_mid,airfoil_mid);
//     }
// //      Single look-up, viscous effects on
// //     elseif strcmpi(oper.toggle_visc,'on') && strcmpi(options.toggle_precompute,'off')
// //         [c_l,sc]    =       lift_coeff_lookup(alpha_0,Re_mid,airfoil_mid);     
// //         [c_d]       =       drag_coeff_lookup(c_l,alpha_0,Re_mid,airfoil_mid,sc);
// //         [c_m]       =       moment_coeff_lookup(alpha_0,Re_mid,airfoil_mid,sc);
// //      Special option for optimization   
// //     elseif strcmpi(oper.toggle_visc,'on') && strcmpi(options.toggle_precompute,'opt')
// //         load('opt_coeffs')
// //         [c_l,c_d,c_m] = airfoil_lookup_for_optimization(alpha_0,Re_mid,airfoil_mid,opt_coeffs);
// //      Viscous effects off
// //     else
// //         alpha_0     =       alpha_0 - oper.alpha_zero;
// //         c_l         =       alpha_0.*oper.a_0;
// //         c_d         =       0;
// //         c_m         =       0;
// //     end
    
// //  Sweep corrected lift and drag. Johnson corrected
//     double Lambda_yaw  =  math::atan((mu_x*math::cos(azimuth))/(r_mid+mu_x*math::sin(azimuth)));
//     //      [c_l,sc]    =       lift_coeff_lookup(alpha_0.*cos(Lambda_yaw).^2,Re_mid,airfoil_mid);
//     //  c_l         =       (c_l)./cos(Lambda_yaw).^2;
//     //     % c_d         =       (drag_coeff_lookup(c_l,alpha_0.*cos(Lambda_yaw),Re_mid,airfoil_mid,sc))./cos(Lambda_yaw); 

// //  Blade forces             
            
// //      Spanwise elemental lift, drag, pitching moment as a function of azimuth position
// //         dL          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_l;
// //         dD          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_d;
// //         dM          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.^2.*c_m;

// //      Spanwise elemental blade forces and moments  
// //         % Thrust (integrated from R(1) to tip)
// //         % Torque (integrated from rotational axis, R = 0, to tip)
// //         % Longitudinal force (normal force, "P-factor")
// //         % Lateral force (side force)
// //         % Rolling moment
// //         % Pitching moment
      
// //         dT          =       B.*(dL.*cos(phi)-(dD.*sin(phi)));
// //         dQ          =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*mid_span;
// //         dP          =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*omega_mid;

// //         dNx         =       B.*-(dL.*sin(phi)+(dD.*cos(phi))).*sin(azimuth);
// //         dNy         =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*cos(azimuth);
// //         dMx         =       B.*-(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*sin(azimuth) + dM.*cos(azimuth);
// //         dMy         =       B.*(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*cos(azimuth) - dM.*sin(azimuth);

// //      Induced drag and decomposed power sources ( Induced, Parasite)
// //         dD_i        =       dL.*sin(phi);                  % Induced drag
// //         dP_i        =       B.*(dL.*sin(phi)).*omega_mid;  % Induced power
// //         dP_p        =       B.*(dD.*cos(phi)).*omega_mid;  % Profile power

// //     % Small angle approximations for the elemental blade forces
// //         dT_small    =       B.*(dL);
// //         dP_small    =       B.*(dL.*sin(phi)+(dD)).*omega_mid;
// //         dP_p_small  =       B.*(dD).*omega_mid;

// //         % Post stall, T is positive? must fix
// //         %if exist('state') == 1 && strcmpi(state,'windmill') == 1 % Recalculate if in windmill mode
// //         %dT              =       B.*(dL.*cos(phi)+(dD.*sin(phi)));
// //         %dQ              =       B.*(dL.*sin(phi)-(dD.*cos(phi))).*mid_span;
// //         %dP              =       B.*(dL.*sin(phi)-(dD.*cos(phi))).*omega_mid;
// //         %end

// //      Blade forces averaged over full rotation
// //         % Thrust (integrated from R(1) to tip)
// //         % Torque (integrated from rotational axis, R = 0, to tip)
// //         % Longitudinal force (normal force)
// //         % Lateral force (side force)
// //         % Rolling moment
// //         % Pitching moment
        
// //         T           =       (sum(simps(mid_span,dT)+simps([mid_span(stations-1);R],[dT(stations-1,:);zeros(1,azm)])))./azm;
// //         Q           =       (sum(simps(mid_span,dQ)+simps([0;mid_span(1)],[zeros(1,azm);dQ(1,:)])+simps([mid_span(stations-1);R],[dQ(stations-1,:);zeros(1,azm)])))./azm;
// //         Nx          =       (sum(simps(mid_span,dNx)+simps([0;mid_span(1)],[zeros(1,azm);dNx(1,:)])+simps([mid_span(stations-1);R],[dNx(stations-1,:);zeros(1,azm)])))./azm;
// //         Ny          =       (sum(simps(mid_span,dNy)+simps([0;mid_span(1)],[zeros(1,azm);dNy(1,:)])+simps([mid_span(stations-1);R],[dNy(stations-1,:);zeros(1,azm)])))./azm;
// //         Mx          =       (sum(simps(mid_span,dMx)+simps([0;mid_span(1)],[zeros(1,azm);dMx(1,:)])+simps([mid_span(stations-1);R],[dMx(stations-1,:);zeros(1,azm)])))./azm;
// //         My          =       (sum(simps(mid_span,dMy)+simps([0;mid_span(1)],[zeros(1,azm);dMy(1,:)])+simps([mid_span(stations-1);R],[dMy(stations-1,:);zeros(1,azm)])))./azm;

// //         % Vector sum of in-plane forces and azimuth direction (in rads) 
// //         Nsum        =       sqrt(Nx^2+Ny^2);
// //         N_angle     =       atan2(Ny,Nx);                   % [Rads]
        
// //         % Total power and decomposed power sources (Total, Induced, Parasite)
// //         P           =       (sum(simps(mid_span,dP)+simps([0;mid_span(1)],[zeros(1,azm);dP(1,:)])+simps([mid_span(stations-1);R],[dP(stations-1,:);zeros(1,azm)])))./azm;
// //         P_i         =       (sum(simps(mid_span,dP_i)+simps([0;mid_span(1)],[zeros(1,azm);dP_i(1,:)])+simps([mid_span(stations-1);R],[dP_i(stations-1,:);zeros(1,azm)])))./azm;
// //         P_p         =       (sum(simps(mid_span,dP_p)+simps([0;mid_span(1)],[zeros(1,azm);dP_p(1,:)])+simps([mid_span(stations-1);R],[dP_p(stations-1,:);zeros(1,azm)])))./azm;
    
// //         % Induced drag
// //         D_i         =       (sum(simps(mid_span,dD_i)+simps([0;mid_span(1)],[zeros(1,azm);dD_i(1,:)])+simps([mid_span(stations-1);R],[dD_i(stations-1,:);zeros(1,azm)])))./azm;

// //         % "Small angle" approximations
// //         T_small     =       (sum(simps(mid_span,dT_small)+simps([0;mid_span(1)],[zeros(1,azm);dT_small(1,:)])+simps([mid_span(stations-1);R],[dT_small(stations-1,:);zeros(1,azm)])))./azm;       
// //         P_small     =       (sum(simps(mid_span,dP_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_small(1,:)])+simps([mid_span(stations-1);R],[dP_small(stations-1,:);zeros(1,azm)])))./azm;       
// //         P_p_small   =       (sum(simps(mid_span,dP_p_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_p_small(1,:)])+simps([mid_span(stations-1);R],[dP_p_small(stations-1,:);zeros(1,azm)])))./azm;

// // %% Rotor/propeller coefficients    
// //     % Rotor force coefficients averaged over full rotation, rotor convention
// //         CT          =       T./(flow.rho.*(pi*R^2)*(omega*R)^2);     % Thrust, rotor
// //         CQ          =       Q./(flow.rho.*(pi*R^2)*(omega*R)^2*R);   % Torque, rotor
// //         CP          =       P./(flow.rho.*(pi*R^2)*(omega*R)^3);     % Power, rotor
// //         CNx         =       Nx./(flow.rho.*(pi*R^2)*(omega*R)^2);    % Normal longitudinal force x-dir, rotor
// //         CNy         =       Ny./(flow.rho.*(pi*R^2)*(omega*R)^2);    % Side lateral force y-dir, rotor
// //         CMx         =       Mx./(flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Rolling moment, rotor
// //         CMy         =       My./(flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Pitching moment, rotor
    


}
void GazeboMotorModel::UniformMomentumFF(Blade* _blade,Flow* _flow){
_flow->vi = 0; _flow-> ui = 0;
_flow->V_R = (pow((_flow->Vrp_a +_flow->vi),2.0) + (_flow->Vaz_perp.transpose().replicate(_blade->omega_Mid.rows(),1) + _blade->omega_Mid.replicate(1, _flow->Vaz_perp.rows()) - _flow->ui).square()).sqrt();

// std::cout<<" V_R = "<< _flow->V_R << std::endl<< std::endl;
std::cout<<"  check1 =\n "<<  _flow->Vaz_perp.transpose().replicate(_blade->omega_Mid.rows(),1) << std::endl<< std::endl;
std::cout<<"  check2 =\n "<<  _blade->omega_Mid.replicate(1, _flow->Vaz_perp.rows()) << std::endl<< std::endl;

std::cout<<"  check3 =\n "<<  _flow->Vaz_perp.transpose().replicate(_blade->omega_Mid.rows(),1) + _blade->omega_Mid.replicate(1, _flow->Vaz_perp.rows()) - _flow->ui<< std::endl<< std::endl;
std::cout<<"  check4 =\n "<<  (_flow->Vaz_perp.transpose().replicate(_blade->omega_Mid.rows(),1) + _blade->omega_Mid.replicate(1, _flow->Vaz_perp.rows()) - _flow->ui).square()<< std::endl<< std::endl;

// std::cout<<"  flow->Vaz_perp^T col 1=\n "<< _flow->Vaz_perp.transpose().cols() << std::endl<< std::endl;
// std::cout<<"  _blade->omega_Mid row 19=\n "<< _blade->omega_Mid.rows() << std::endl<< std::endl;
// std::cout<<"  _blade->omega_Mid col 1=\n "<< _blade->omega_Mid.cols() << std::endl<< std::endl;



//  std::cout<< _flow->Vaz_perp << std::endl<< std::endl;
//  std::cout<< _blade->omega_Mid << std::endl<< std::endl;

// _flow->Re_Mid = 
// double* coeff_AF15 = coeffLookup(_flow->Re_Mid, _flow->inflow_Angle ,_blade->airfoil15_database) ;


}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}
