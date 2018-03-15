/* © Ascento; focusproject 2017/2018; ASL ETH Zurich
___   _____ _____________   _____________
/   | / ___// ____/ ____/ | / /_  __/ __  |
/ /| | \__ \/ /   / __/ /  |/ / / / / / / /
/ ___ |___/ / /___/ /___/ /|  / / / / /_/ /
/_/  |_/____/\____/_____/_/ |_/ /_/  \____/
Author: Lionel Gulich <lgulich@student.ethz.ch> - February 2018
--------------------------------------------------------------------------------
Code provided under the GNU General Public Licence GNU GPLv3
--------------------------------------------------------------------------------
This code provides the framework for the stabilisation control of the sara
prototype. It uses multiple lqr controllers derived from different equilibrium
positions of the system. Between the positions the controllers are interpolated.
For read in of the k matrix array the matrices should be stored in "/resources"
as a .csv file and named as "k_matrix_0", "k_matrix_1", ...
For read in of the imu_offset scaler array the vectors should be stored
in "/resources" as a .csv file and named as "imu_offset_0", "imu_offset_1", ...
--------------------------------------------------------------------------------
TODO: is double necessary in matrices or would float be enough?
--------------------------------------------------------------------------------
*/

//Include header, ros and other directories
#include <ros/ros.h>
#include "stab_controller_multi_lqr_node.h"

//Set namespace
namespace ascento {

  //Constructor / Destructor
  StabControllerMultiLqrNode::StabControllerMultiLqrNode(ros::NodeHandle& nh) : maxon_left(1,"USB1", "EPOS4"), maxon_right(2,"USB0", "EPOS4"), nh(nh) {
    ROS_INFO("stab_controller_multi_lqr node has started.");

    //Initialise class members
    Initialise();

    //Initialise Threads for reading position/velocity and writing torques to maxon motors
    std::thread ReadLeft(&StabControllerMultiLqrNode::ReadLeft,this);
    std::thread ReadRight(&StabControllerMultiLqrNode::ReadRight,this);
    std::thread WriteLeft(&StabControllerMultiLqrNode::WriteLeft,this);
    std::thread WriteRight(&StabControllerMultiLqrNode::WriteRight,this);

    // Start rated loop
    RateController();

  }
  StabControllerMultiLqrNode::~StabControllerMultiLqrNode() {
    maxon_left.MoveWithTorque(0);
    maxon_right.MoveWithTorque(0);
  }

  //Setup methods
  void StabControllerMultiLqrNode::ResetStates() {
    encoder_offset_left = maxon_left.GetPosition()*2*M_PI/encoder_quadcounts;
    encoder_offset_right = maxon_right.GetPosition()*2*M_PI/encoder_quadcounts;
  }
  void StabControllerMultiLqrNode::Initialise() {
    // Read parameters from server
    if (!ReadParameters()) {
      ROS_ERROR("Could not read parameters.");
      ros::requestShutdown();
    }

    InitSubPub();
    InitSrv();

    // Setup Controller
    ResetStates();
    Eigen::VectorXd u_sat = Eigen::MatrixXd(NO_OF_INPUTS,1);
    u_sat << saturation_ec90;
    controller.SetUSat(u_sat);
    input_delays_left.setZero(NO_OF_DELAYS, 1);
    input_delays_right.setZero(NO_OF_DELAYS, 1);
    input_left.setZero(NO_OF_INPUTS, 1);
    input_right.setZero(NO_OF_INPUTS, 1);
    meas_left.setZero(NO_OF_MEASUREMENTS, 1);
    meas_right.setZero(NO_OF_MEASUREMENTS, 1);


    // Read in K matrices
    ReadInMatrixArray("/home/ascento/catkin_ws/src/ascento/Matlab/DynMod/fixed_drive/ros_compatible/k_matrix_");
    // Read in Imu offsets
    ReadInDoubleArray("/home/ascento/catkin_ws/src/ascento/Matlab/DynMod/fixed_drive/ros_compatible/imu_offset");
  }
  void StabControllerMultiLqrNode::InitSubPub() {
    //Initialise subscribers
    imu_sub = nh.subscribe("/est_states/imu", 1, &StabControllerMultiLqrNode::SaveImuCallback, this);
    anydrive_left_sub = nh.subscribe("/sens_states/hip_mot/left", 1, &StabControllerMultiLqrNode::SaveAnydriveLeftCallback, this);
    anydrive_right_sub = nh.subscribe("/sens_states/hip_mot/right", 1, &StabControllerMultiLqrNode::SaveAnydriveRightCallback, this);

    maxon_execution_pub = nh.advertise<ascento_msgs::wheel_input_torque>("/actuate/wheel_mot", 1);
  }
  void StabControllerMultiLqrNode::InitSrv() {
    //Initialise services
    controller_starter = nh.advertiseService("/stab_control_start", &StabControllerMultiLqrNode::StartStopCallback, this);
    imu_autotune = nh.advertiseService("/set_imu_offset", &StabControllerMultiLqrNode::SetImuOffsetCallback, this);
  }
  bool StabControllerMultiLqrNode::ReadParameters() {
    // Geometric parameters
    if (!nh.getParam("/wheel/radius", wheel_radius)) return false;
    if (!nh.getParam("/phi/min", phi_min)) return false;
    if (!nh.getParam("/phi/max", phi_max)) return false;
    if (!nh.getParam("/maxon_ec90_max_torque", saturation_ec90)) return false;
    if (!nh.getParam("/encoder/quadcounts", encoder_quadcounts)) return false;
    if (!nh.getParam("/measured_imu_offset", measured_imu_offset)) ROS_INFO("No param \"/measured_imu_offset\" found, run srv \"/set_imu_offset\"");

    // Controller Parameters
    if (!nh.getParam("/stab_controller_multi_lqr/sampling_rate", sampling_rate)) return false;

    return true;
  }

  // Threading methods maxon motors
  void StabControllerMultiLqrNode::ReadLeft(){
    ros::Rate rate_ReadLeft(sampling_rate);
    while (ros::ok()) {
      if(is_running){
        std::cout << "/* readleft */" << '\n';
        meas_left(2) = (maxon_left.GetPosition()*2.0*M_PI/encoder_quadcounts-encoder_offset_left);
        meas_left(3) = maxon_left.GetAverageVelocity()*2.0*M_PI/60.0;
        rate_ReadLeft.sleep();
      }
    }
  }
  void StabControllerMultiLqrNode::ReadRight(){
    ros::Rate rate_ReadRight(sampling_rate);
    while (ros::ok()) {
      if(is_running){
        std::cout << "/* readright */" << '\n';
        meas_right(2) = (-maxon_right.GetPosition()*2.0*M_PI/encoder_quadcounts-encoder_offset_right);
        meas_right(3) = -maxon_right.GetAverageVelocity()*2.0*M_PI/60.0;
        rate_ReadRight.sleep();
      }
    }
  }
  void StabControllerMultiLqrNode::WriteLeft(){
    ros::Rate rate_WriteLeft(sampling_rate);
    while (ros::ok()) {
      if(is_running){
        std::cout << "/* writeleft */" << '\n';
        maxon_left.MoveWithTorque(input_left(0));
        rate_WriteLeft.sleep();
      }
    }
  }
  void StabControllerMultiLqrNode::WriteRight(){
    ros::Rate rate_WriteRight(sampling_rate);
    while (ros::ok()) {
      if(is_running){
        std::cout << "/* writeright */" << '\n';
        maxon_right.MoveWithTorque(-input_right(0));
        rate_WriteRight.sleep();
      }
    }
  }

  //Callback function
  void StabControllerMultiLqrNode::SaveImuCallback(const sensor_msgs::Imu &msg){
    //Create an object of type tf::Quaternion
    tf::Quaternion q_body;

    //Convert geometry_msgs::Quaternion to tf::Quaternion
    tf::quaternionMsgToTF(msg.orientation, q_body);

    //Matrix needed for rpy angle extraction out of quaternion
    tf::Matrix3x3 m_body(q_body);

    //Convert quaternion to rpy angles (only pitch is needed)
    double roll_body, pitch_body, yaw_body;
    m_body.getRPY(roll_body, pitch_body, yaw_body);

    //Read body velocity and save it globally
    meas_left(0) = pitch_body;
    meas_left(1) = msg.angular_velocity.y;
    meas_right(0) = pitch_body;
    meas_right(1) = msg.angular_velocity.y;

  }
  void StabControllerMultiLqrNode::SaveAnydriveLeftCallback(const series_elastic_actuator_msgs::SeActuatorReadingExtended &msg){
    phi_left = msg.state.joint_position;
  }
  void StabControllerMultiLqrNode::SaveAnydriveRightCallback(const series_elastic_actuator_msgs::SeActuatorReadingExtended &msg){
    phi_right = msg.state.joint_position;
  }
  bool StabControllerMultiLqrNode::StartStopCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {

    if(is_running!=request.data) { // check if request is not equal to current status
      is_running = request.data;
      if(is_running) {
        ResetStates();
        response.message = "stab controller started";
        response.success = true;
        ROS_INFO("Stab controller started");
      }
      else{
        response.message = "stab controller stopped";
        response.success = true;
        maxon_left.MoveWithTorque(0);
        maxon_right.MoveWithTorque(0);

        // wait and repeat to be sure it executes
        ros::Duration(0.5).sleep();
        maxon_left.MoveWithTorque(0);
        maxon_right.MoveWithTorque(0);
        ROS_INFO("Stab controller stopped");
      }

      return true;
    }

    else if(is_running==request.data) { // requested status already in execution
      ROS_WARN("Redundant service input for starting controller. No action taken");
      response.success = false;
      response.message = "request redundant - no action taken";
      return false;
    }

  }
  bool StabControllerMultiLqrNode::SetImuOffsetCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    int i=100;
    double offset=0;
    while(i--) {
      offset += meas_left(0);
      ros::Duration(0.1).sleep();
    }

    measured_imu_offset = offset/100;
    nh.setParam("/measured_imu_offset", measured_imu_offset);
    std::ostringstream oss;
    oss << "Imu offset succesfully set to " << measured_imu_offset << ". Please set parameter in yaml accordingly";
    response.message = oss.str();
    response.success = true;
  }

  //Control function
  void StabControllerMultiLqrNode::RateController(){
    //Set Frequency of controller
    ros::Rate r(sampling_rate);
    while(ros::ok()) {
      ros::spinOnce();
      if(is_running) {
        ControllerFunction();
        r.sleep();
      }
    }
  }
  void StabControllerMultiLqrNode::ControllerFunction(){

    // Get mean anydrive angle
    double phi_left = phi_max;
    double phi_right = phi_max;
    double phi_mean = phi_max;//(phi_left + phi_right)/2;

    // Invert Measurement equations to get states
    state_left(0) = meas_left(0)+LipImuOffset(phi_left)-measured_imu_offset;
    state_left(1) = meas_left(1);
    state_left(2) = meas_left(2)+state_left(0);
    state_left(3) = meas_left(3)+state_left(1);

    state_right(0) = meas_right(0)+LipImuOffset(phi_right)-measured_imu_offset;
    state_right(1) = meas_right(1);
    state_right(2) = meas_right(2)+state_right(0);
    state_right(3) = meas_right(3)+state_right(1);

    // Allocate total state vector, consisting of dynamic states and delayed input
    Eigen::Matrix<double, NO_OF_STATES+NO_OF_DELAYS, 1> state_left_total;
    Eigen::Matrix<double, NO_OF_STATES+NO_OF_DELAYS, 1> state_right_total;

    // fill total state vector with dynamic states and delayed inputs
    state_left_total << state_left, input_delays_left;
    state_right_total << state_right, input_delays_right;

    // Set K matrix of controller according to current phi
    LipKMatrix(phi_mean);

    // Get newest control commands
    controller.UpdateControls(state_left_total, input_left);
    controller.UpdateControls(state_right_total, input_right);
    std::cout << "/* controller */" << '\n';

    // shift delayed input vector and append newest input
    input_delays_left << input_delays_left.tail(NO_OF_DELAYS-1), input_left(0);
    input_delays_right << input_delays_right.tail(NO_OF_DELAYS-1), input_right(0);


    //Debugging outputs
    //std::cout<<"input_left:"<<input_left<<std::endl;
    //std::cout<<"input_right:"<<input_right<<std::endl;
    //controller.PrintK();
    //std::cout << "states_left" << state_left << '\n';
    //std::cout << "states_right" << state_right << '\n';

    // Publish maxon actutation inputs for debugging frequency
    ascento_msgs::wheel_input_torque output_msg;
    output_msg.torque_left = -input_left(0);
    output_msg.torque_right = input_left(0);
    maxon_execution_pub.publish(output_msg);
  }

  // Matrix Interpolation
  void StabControllerMultiLqrNode::LipKMatrix(const double phi){
    // method interpolates linearly betwen nearest K-matrices
    int index1;
    int index2;
    double phi_normalised;

    if (phi<phi_min) {
      index1 = 0;
      index2 = 0;
      ROS_WARN("LIP: phi out of bounds, using phi min");

    } else if (phi>phi_max) {
      index1 = NO_OF_EQUILIBRIUM_POSITIONS-1;
      index2 = NO_OF_EQUILIBRIUM_POSITIONS-1;
      ROS_WARN("LIP: phi out of bounds, using phi max");

    } else {
      // map theta to [0, 1]:
      phi_normalised = (phi-phi_min)/(phi_max-phi_min);
      // find nearest k matrices
      index1 = floor((NO_OF_EQUILIBRIUM_POSITIONS-1)*phi_normalised);
      index2 = ceil((NO_OF_EQUILIBRIUM_POSITIONS-1)*phi_normalised);
    }

    // actual interpolation
    controller.SetK(k_matrix[index1] + (k_matrix[index2]-k_matrix[index1]) * (9*phi_normalised-index1));

  }
  double StabControllerMultiLqrNode::LipImuOffset(const double phi){
    // method interpolates linearly betwen nearest K-matrices
    int index1;
    int index2;
    double phi_normalised;

    if (phi<phi_min) {
      index1 = 0;
      index2 = 0;
      ROS_WARN("LIP: phi out of bounds, using phi min");

    } else if (phi>phi_max) {
      index1 = NO_OF_EQUILIBRIUM_POSITIONS-1;
      index2 = NO_OF_EQUILIBRIUM_POSITIONS-1;
      ROS_WARN("LIP: phi out of bounds, using phi max");

    } else {
      // map theta to [0, 1]:
      phi_normalised = (phi-phi_min)/(phi_max-phi_min);
      // find nearest k matrices
      index1 = floor((NO_OF_EQUILIBRIUM_POSITIONS-1)*phi_normalised);
      index2 = ceil((NO_OF_EQUILIBRIUM_POSITIONS-1)*phi_normalised);
    }
    // actual interpolation
    return imu_offset[index1] + (imu_offset[index2]-imu_offset[index1])*((NO_OF_EQUILIBRIUM_POSITIONS-1)*phi_normalised-index1);
  }

  // Matrix read in methods
  void StabControllerMultiLqrNode::ReadInMatrixArray(std::string base_filename){
    // read in k matrices in to k_matrix array
    for (int i = 0; i < NO_OF_EQUILIBRIUM_POSITIONS; i++) {
      k_matrix[i] = ReadMatFromCSV(base_filename+std::to_string(i)+".csv", NO_OF_INPUTS, (NO_OF_STATES+NO_OF_DELAYS));
    }
  }
  void StabControllerMultiLqrNode::ReadInDoubleArray(std::string base_filename){
    // read in imu offset values in to imu_offset array
    Eigen::VectorXd imu_offset_vector = ReadMatFromCSV(base_filename+".csv", NO_OF_EQUILIBRIUM_POSITIONS, 1);
    for (int i = 0; i < NO_OF_EQUILIBRIUM_POSITIONS; i++) {
      imu_offset[i] = imu_offset_vector(i);
    }
  }
  Eigen::MatrixXd StabControllerMultiLqrNode::ReadMatFromCSV(std::string file, int rows, int cols) {
    // method reads in Eigen::Matrix from .csv file
    std::ifstream fin(file);
    std::string line;
    int row = 0; //index variable
    int col = 0; //index variable
    Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

    if (fin.is_open()) {
      while (std::getline(fin, line)) {
        char *ptr = (char *) line.c_str();
        int len = line.length();
        col = 0;
        char *start = ptr;

        for (int i = 0; i < len; i++) {
          if (ptr[i] == ',') {
            res(row, col++) = atof(start);
            start = ptr + i + 1;
          }


        }

        res(row, col) = atof(start);
        row++;
      }
      fin.close();
    }
    else {
      std::cerr << "/* File not found */ " << file << '\n';
    }
    return res;
  }
} /* namespace */

// Main function
int main(int argc, char **argv){
  ros::init(argc, argv, "stab_controller_multi_lqr");
  ros::NodeHandle nh("");

  ascento::StabControllerMultiLqrNode stab_controller_multi_lqr(nh);

  ros::spin();

  return 0;
}
