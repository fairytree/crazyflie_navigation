//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    Tutorial controller to teach Eigen and OSQP on a MPC controller
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These various headers need to be included so that this controller service can be
// connected with the D-FaLL system.

//some useful libraries
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
//#include "dfall_pkg/StringWithHeader.h"
#include "dfall_pkg/SetpointWithHeader.h"
#include "dfall_pkg/CustomButtonWithHeader.h"
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/Setpoint.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/DebugMsg.h"

// Include the DFALL service types
#include "dfall_pkg/LoadYamlFromFilename.h"
#include "dfall_pkg/GetSetpointService.h"
//#include "dfall_pkg/GetDebugValuesService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>

// Include Eigen for matrix operations
#include "Eigen/Dense"
#include "Eigen/Sparse"

// Include OSQP optimization platform
#include "osqp.h"


// Namespacing the package
using namespace dfall_pkg;
using namespace Eigen;




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.



// NOTE: these constants are already defined in the "Constant.h" header file
//       and are repeated here for convenience

// These constants define the modes that can be used for controller this is
// running on-board the Crazyflie 2.0.
// Therefore, the constants defined here need to be in agreement with those
// defined in the firmware running on-board the Crazyflie 2.0.
// The following is a short description about each mode:
//
// CF_COMMAND_TYPE_MOTORS
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors
//
// CF_COMMAND_TYPE_RATE
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors, and additionally request the
//     body frame roll, pitch, and yaw angular rates from the PID rate
//     controllers implemented in the Crazyflie 2.0 firmware.
//
// CF_COMMAND_TYPE_ANGLE
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors, and additionally request the
//     body frame roll, pitch, and yaw angles from the PID attitude
//     controllers implemented in the Crazyflie 2.0 firmware.
//#define CF_COMMAND_TYPE_MOTORS 6
//#define CF_COMMAND_TYPE_RATE   7
//#define CF_COMMAND_TYPE_ANGLE  8




// *** CONSTANTS ***
const float const_gravity = 9.81;
const int const_num_states = 8;		// not controlling yaw with MPC
const int const_num_inputs = 3;		// not controlling yaw with MPC
const int const_num_states_plus_inputs = const_num_states + const_num_inputs;



//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;

// NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;



// *** VARAIBLES FOR VALUES LOADED FROM THE YAML FILE ***
// > the mass of the crazyflie, in [grams]
float yaml_cf_mass_in_grams = 30.0;

// The weight of the Crazyflie in Newtons, i.e., mg
float m_cf_weight_in_newtons = yaml_cf_mass_in_grams * const_gravity / 1000.0;

// > the frequency at which the controller is running
float yaml_control_frequency = 200.0;

// > the default setpoint, the ordering is (x, y, z),
//   with units [meters, meters, meters]
std::vector<float> yaml_default_setpoint = {0.0, 0.0, 0.4};


// > the MPC prediction horizon, in discrete time steps
int yaml_prediction_horizon = 100;

// > the MPC state cost matrix diagonal entries, the ordering is (x, y, z, xdot, ydot, zdot, roll, pitch)
std::vector<float> yaml_state_cost_diagonals = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// > the MPC input cost matrix diagonal entries, the ordering is (totalThrust, rollRate, pitchRate)
std::vector<float> yaml_input_cost_diagonals = {1.0, 1.0, 1.0};

// > the MPC state constraints, the ordering is (x, y, z, xdot, ydot, zdot, roll, pitch)
std::vector<float> yaml_min_state_constraints = {-2.0, -2.0, 0.1, -100, -100, -100, -0.5236, -0.5236};
std::vector<float> yaml_max_state_constraints = {2.0, 2.0, 3.0, 100, 100, 100, 0.5236, 0.5236};

// > the MPC input constraints, the ordering is (totalThrust, rollRate, pitchRate)
std::vector<float> yaml_min_input_constraints = {0.1597, -1.5708, -1.5708};
std::vector<float> yaml_max_input_constraints = {0.4791, 1.5708, 1.5708};



// *** LQR CONTROLLER PARAMETERS FOR BACKUP CONTROLLER ***
std::vector<float> m_gainMatrixRollRate    =  { 0.00,-4.00, 0.00, 0.00,-1.00, 0.00, 5.00, 0.00, 0.00};
std::vector<float> m_gainMatrixPitchRate   =  { 4.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 5.00, 0.00};
std::vector<float> m_gainMatrixYawRate     =  { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.00};
std::vector<float> m_gainMatrixThrust      =  { 0.00, 0.00, 0.50, 0.00, 0.00, 0.25, 0.00, 0.00, 0.00};



// *** VARIABLES USED FOR THE MPC OPTIMIZATION PROBLEM WITH OSQP ***
// flag to indicate if the MPC optimization problem was setup successfully with OSQP
bool m_mpc_optimization_setup_success = false;

// the state cost matrix
MatrixXf m_state_cost_matrix;

// the current state vector
MatrixXf m_current_state_vector = MatrixXf::Zero(const_num_states, 1);

// the state setpoint to be tracked
MatrixXf m_state_setpoint_vector = MatrixXf::Zero(const_num_states, 1);

// the OSQP linear cost vector as Eigen matrix
MatrixXf m_osqp_q_vector;

// the OSQP linear cost vector for one time step as Eigen matrix
MatrixXf m_osqp_q_vector_for_one_time_step;

// the OSQP linear cost vector as c_float array
c_float* m_osqp_q_runtime_cfloat;

// the OSQP inequality constraint lower bound vector as c_float array
c_float* m_osqp_l_runtime_cfloat;

// the OSQP inequality constraint upper bound vector as c_float array
c_float* m_osqp_u_runtime_cfloat;

// the osqp workspace
OSQPWorkspace* m_osqp_work;

// the input vector to apply
MatrixXf m_mpc_input_vector_to_apply = MatrixXf::Zero(const_num_inputs, 1);



// *** MISCELLANEOUS ***
// The location error of the Crazyflie at the "previous" time step
float m_previous_stateErrorInertial[9];

// The setpoint to be tracked, the ordering is (x, y, z, yaw),
// with units [meters, meters, meters, radians]
std::vector<float>  m_setpoint{0.0, 0.0, 0.4, 0.0};

// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;

// ROS Publisher to inform the network about
// changes to the setpoint
ros::Publisher m_setpointChangedPublisher;

// ROS Publisher to inform the network about changes to the optimization setup status
ros::Publisher m_optimizationSetupStatusChangedPublisher;







//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    PPPP   RRRR    OOO   TTTTT   OOO   TTTTT  Y   Y  PPPP   EEEEE   SSSS
//    P   P  R   R  O   O    T    O   O    T     Y Y   P   P  E      S
//    PPPP   RRRR   O   O    T    O   O    T      Y    PPPP   EEE     SSS
//    P      R  R   O   O    T    O   O    T      Y    P      E          S
//    P      R   R   OOO     T     OOO     T      Y    P      EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These function prototypes are not strictly required for this code to
// complile, but adding the function prototypes here means the the functions
// can be written below in any order. If the function prototypes are not
// included then the function need to written below in an order that ensure
// each function is implemented before it is called from another function,
// hence why the "main" function is at the bottom.

// CONTROLLER COMPUTATIONS
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR
// INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived);

// FOR LOADING THE YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyTutorialControllerYamlCallback(const IntWithHeader & msg);
void fetchTutorialControllerYamlParameters(ros::NodeHandle& nodeHandle);



// *** SETUP THE MPC OPTIMIZATION PROBLEM WITH OSQP ***
void setup_mpc_optimization();

// *** CHANGE THE MPC OPTIMIZATION SETPOINT ***
void change_mpc_optimization_setpoint();

// *** UPDATE INITIAL CONDITION AND SOLVE OPTIMIZATION ***
bool update_initial_condition_and_solve_mpc_optimization();

// *** HELPER FUNCTIONS ***
// Converts Eigen Dense matrix to CSC format used in OSQP
csc* eigen2csc(const MatrixXf& eigen_dense_mat);

// COMMUNICATE OPTIMIZATION SETUP STATUS TO GUI
void send_mpc_optimization_setup_status_to_gui();

// OSQP extended cleanup
void osqp_extended_cleanup();

// OSQP data cleanup
void osqp_cleanup_data(OSQPData* data);