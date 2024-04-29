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
//    Place for students to implement their controller
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
#include "dfall_pkg/FloatWithHeader.h"
//#include "dfall_pkg/StringWithHeader.h"
#include "dfall_pkg/SetpointWithHeader.h"
//#include "dfall_pkg/CustomButtonWithHeader.h"
#include "dfall_pkg/ViconData.h"
//#include "dfall_pkg/Setpoint.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/DebugMsg.h"

// Include the DFALL service types
#include "dfall_pkg/LoadYamlFromFilename.h"
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"




// //the generated structs from the msg-files have to be included
// #include "dfall_pkg/ViconData.h"
// #include "dfall_pkg/Setpoint.h"
// #include "dfall_pkg/ControlCommand.h"
// #include "dfall_pkg/Controller.h"
// #include "dfall_pkg/DebugMsg.h"
// #include "dfall_pkg/CustomButton.h"
// #include "dfall_pkg/ViconSubscribeObjectName.h"

// // Include the Parameter Service shared definitions
// #include "nodes/Constants.h"

// #include <std_msgs/Int32.h>




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

// These constants define the controller used for computing the response in the
// "calculateControlOutput" function
// The following is a short description about each mode:
//
// LQR_MODE_MOTOR     LQR controller based on the state vector:
//                    [position,velocity,angles,angular velocity]
//                    commands per motor thrusts
//
// LQR_MODE_ACTUATOR  LQR controller based on the state vector:
//                    [position,velocity,angles,angular velocity]
//                    commands actuators of total force and bodz torques
//
// LQR_MODE_RATE      LQR controller based on the state vector:
//                    [position,velocity,angles]
//
// LQR_MODE_ANGLE     LQR controller based on the state vector:
//                    [position,velocity]
//
//#define CONTROLLER_MODE_LQR_MOTOR               1
//#define CONTROLLER_MODE_LQR_ACTUATOR            2
#define CONTROLLER_MODE_LQR_RATE                3   // (DEFAULT)
#define CONTROLLER_MODE_LQR_ANGLE               4
#define CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED   5
//#define CONTROLLER_MODE_ANGLE_RESPONSE_TEST     6


// These constants define the method used for estimating the Inertial
// frame state.
// All methods are run at all times, this flag indicates which estimate
// is passed onto the controller.
// The following is a short description about each mode:
//
// ESTIMATOR_METHOD_FINITE_DIFFERENCE
//       Takes the poisition and angles directly as measured,
//       and estimates the velocities as a finite different to the
//       previous measurement
//
// ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION
//       Uses a 2nd order random walk estimator independently for
//       each of (x,y,z,roll,pitch,yaw)
//
// ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED
//       Uses the model of the quad-rotor and the previous inputs
//
#define ESTIMATOR_METHOD_FINITE_DIFFERENCE          1
#define ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION   2   // (DEFAULT)
#define ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED      3

// These constants define the controller used for computing the response in the
// "calculateControlOutput" function
// The following is a short description about each mode:
// LQR_RATE_MODE      LQR controller based on the state vector:
//                    [position,velocity,angles]
//
// LQR_ANGLE_MODE     LQR controller based on the state vector:
//                    [position,velocity]
//
#define LQR_RATE_MODE   1   // (DEFAULT)
#define LQR_ANGLE_MODE  2

// Namespacing the package
using namespace dfall_pkg;


//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------




float m_gain_P = 0.31;
float m_gain_P_to_D_ratio = 0.6;








// ******************************************************************************* //
// VARIABLES SPECIFIC TO THE TUNING CONTROL FEATURE

// Setpoint for the HORIZONTAL test
int test_horizontal_currentpoint = 1;
std::vector<float> test_horizontal_setpoint1 (4,0.0);
std::vector<float> test_horizontal_setpoint2 (4,0.0);

// Setpoint for the VERTICAL test
int test_vertical_currentpoint = 1;
std::vector<float> test_vertical_setpoint1 (4,0.0);
std::vector<float> test_vertical_setpoint2 (4,0.0);

// Setpoint for the HEADING test
int test_heading_currentpoint = 1;
std::vector<float> test_heading_setpoint1 (4,0.0);
std::vector<float> test_heading_setpoint2 (4,0.0);

// Setpoint for the ALL test
int test_all_currentpoint = 1;
std::vector<float> test_all_setpoint1 (4,0.0);
std::vector<float> test_all_setpoint2 (4,0.0);

// Parameters for flying in a circle
float test_circle_radius = 0.2;
float test_circle_seconds_per_rev = 5.0;
float test_circle_height = 0.4;
float test_circle_pirouette_per_rev = 0.0;
float test_circle_time_to_reach_start = 0.7;
bool isActive_fly_test_circle = false;
int test_circle_ticks_since_start = 0;

// Multipliers for the HORIZONTAL contorller
float multiplier_horizontal = 0.1;
float multiplier_horizontal_min = 0.9;
float multiplier_horizontal_max = 1.1;
// Multipliers for the VERTICAL contorller
float multiplier_vertical = 0.1;
float multiplier_vertical_min = 0.9;
float multiplier_vertical_max = 1.1;
// Multipliers for the HEADING contorller
float multiplier_heading = 0.1;
float multiplier_heading_min = 0.9;
float multiplier_heading_max = 1.1;


// LQR Gain matrix for tracking the angle commands from the Crazyflie
std::vector<float> gainMatrixRollRate_forRemoteControl     (3,0.0);
std::vector<float> gainMatrixPitchRate_forRemoteControl    (3,0.0);
std::vector<float> gainMatrixYawRate_forRemoteControl      (3,0.0);

// ******************************************************************************* //



// VARIABLES FOR SOME "ALMOST CONSTANTS"
// > Mass of the Crazyflie quad-rotor, in [grams]
float cf_mass;
// > Coefficients of the 16-bit command to thrust conversion
std::vector<float> motorPoly(3);
// The weight of the Crazyflie in [Newtons], i.e., mg
float gravity_force;
// One quarter of the "gravity_force"
float gravity_force_quarter;




// VARIABLES FOR THE CONTROLLER

// Frequency at which the controller is running
float yaml_vicon_frequency;

// Frequency at which the controller is running
float control_frequency;


// > The setpoints for (x,y,z) position and yaw angle, in that order
float setpoint[4] = {0.0,0.0,0.4,0.0};




// The controller type to run in the "calculateControlOutput" function
int controller_mode = CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED;

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
std::vector<float> gainMatrixThrust_NineStateVector (9,0.0);
std::vector<float> gainMatrixRollRate               (9,0.0);
std::vector<float> gainMatrixPitchRate              (9,0.0);
std::vector<float> gainMatrixYawRate                (9,0.0);

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE"
std::vector<float> gainMatrixThrust_SixStateVector (6,0.0);
std::vector<float> gainMatrixRollAngle             (6,0.0);
std::vector<float> gainMatrixPitchAngle            (6,0.0);

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED"
std::vector<float> gainMatrixThrust_SixStateVector_50Hz (6,0.0);
std::vector<float> gainMatrixRollAngle_50Hz             (6,0.0);
std::vector<float> gainMatrixPitchAngle_50Hz            (6,0.0);

std::vector<float> gainMatrixRollRate_Nested            (3,0.0);
std::vector<float> gainMatrixPitchRate_Nested           (3,0.0);
std::vector<float> gainMatrixYawRate_Nested             (3,0.0);

int   lqr_angleRateNested_counter = 4;
float lqr_angleRateNested_prev_thrustAdjustment = 0.0;
float lqr_angleRateNested_prev_rollAngle        = 0.0;
float lqr_angleRateNested_prev_pitchAngle       = 0.0;
float lqr_angleRateNested_prev_yawAngle         = 0.0;


// // SAME GAINS AGAIN BUT INSTEAD AS DEFAULTS


// // The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
// std::vector<float> gainMatrixThrust_NineStateVector_default (9,0.0);
// std::vector<float> gainMatrixRollRate_default               (9,0.0);
// std::vector<float> gainMatrixPitchRate_default              (9,0.0);
// std::vector<float> gainMatrixYawRate_default                (9,0.0);

// // The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE"
// std::vector<float> gainMatrixThrust_SixStateVector_default (6,0.0);
// std::vector<float> gainMatrixRollAngle_default             (6,0.0);
// std::vector<float> gainMatrixPitchAngle_default            (6,0.0);

// // The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED"
// std::vector<float> gainMatrixThrust_SixStateVector_50Hz_default (6,0.0);
// std::vector<float> gainMatrixRollAngle_50Hz_default             (6,0.0);
// std::vector<float> gainMatrixPitchAngle_50Hz_default            (6,0.0);

// std::vector<float> gainMatrixRollRate_Nested_default            (3,0.0);
// std::vector<float> gainMatrixPitchRate_Nested_default           (3,0.0);
// std::vector<float> gainMatrixYawRate_Nested_default             (3,0.0);


// The 16-bit command limits
float cmd_sixteenbit_min;
float cmd_sixteenbit_max;


// VARIABLES FOR THE ESTIMATOR

// Frequency at which the controller is running
float m_estimator_frequency;

// > A flag for which estimator to use:
int yaml_estimator_method = ESTIMATOR_METHOD_FINITE_DIFFERENCE;
// > The current state interial estimate,
//   for use by the controller
float m_current_stateInertialEstimate[12];

// > The measurement of the Crazyflie at the "current" time step,
//   to avoid confusion
float m_current_xzy_rpy_measurement[6];

// > The measurement of the Crazyflie at the "previous" time step,
//   used for computing finite difference velocities
float m_previous_xzy_rpy_measurement[6];

// > The full 12 state estimate maintained by the finite
//   difference state estimator
float m_stateInterialEstimate_viaFiniteDifference[12];

// > The full 12 state estimate maintained by the point mass
//   kalman filter state estimator
float m_stateInterialEstimate_viaPointMassKalmanFilter[12];

// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
// > For the (x,y,z) position
std::vector<float> PMKF_Ahat_row1_for_positions (2,0.0);
std::vector<float> PMKF_Ahat_row2_for_positions (2,0.0);
std::vector<float> PMKF_Kinf_for_positions      (2,0.0);
// > For the (roll,pitch,yaw) angles
std::vector<float> PMKF_Ahat_row1_for_angles    (2,0.0);
std::vector<float> PMKF_Ahat_row2_for_angles    (2,0.0);
std::vector<float> PMKF_Kinf_for_angles         (2,0.0);



// VARIABLES FOR THE NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;


// ROS PUBLISHER FOR SENDING OUT THE DEBUG MESSAGES
ros::Publisher m_debugPublisher;


// VARIABLES RELATING TO PERFORMING THE CONVERSION INTO BODY FRAME

// Boolean whether to execute the convert into body frame function
bool shouldPerformConvertIntoBodyFrame = false;


// VARIABLES RELATING TO THE PUBLISHING OF A DEBUG MESSAGE

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool shouldDisplayDebugInfo = false;


// VARIABLES RELATING TO PUBLISHING CURRENT POSITION AND FOLLOWING ANOTHER AGENT'S
// POSITION

// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;





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

// These function prototypes are not strictly required for this code to complile, but
// adding the function prototypes here means the the functions can be written below in
// any order. If the function prototypes are not included then the function need to
// written below in an order that ensure each function is implemented before it is
// called from another function, hence why the "main" function is at the bottom.

// CONTROLLER COMPUTATIONS
// > The function that is called to "start" all estimation and control computations
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);

// > The various functions that implement an LQR controller
void calculateControlOutput_viaLQR(                     float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQRforMotors(            float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQRforActuators(         float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQRforRates(             float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQRforAngles(            float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQRforAnglesRatesNested( float stateErrorBody[12], Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaAngleResponseTest(       float stateErrorBody[12], Controller::Request &request, Controller::Response &response);

// ESTIMATOR COMPUTATIONS
void performEstimatorUpdate_forStateInterial(Controller::Request &request);
void performEstimatorUpdate_forStateInterial_viaFiniteDifference(bool isFirstUpdate);
void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter(bool isFirstUpdate);


// PUBLISHING OF THE DEBUG MESSAGE
void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response);


// TRANSFORMATION FROM INTERIAL ESTIMATE TO BODY FRAME ERROR
void convert_stateInertial_to_bodyFrameError(float stateInertial[12], float setpoint[4], float (&bodyFrameError)[12]);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[12], float (&stateBody)[12], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);




// SETPOINT CHANGE CALLBACK
//void setpointCallback(const Setpoint& newSetpoint);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
//bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);


// REQUEST SETPOINT CHANGE CALLBACK
void requestGainChangeCallback(const FloatWithHeader& newGain);


// FOR LOADING THE YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyTuningControllerYamlCallback(const IntWithHeader & msg);
void fetchTuningControllerYamlParameters(ros::NodeHandle& nodeHandle);


// ******************************************************************************* //
// FUNCTIONS SPECIFIC TO THE TUNING CONTROL FEATURE

// ACTIVATE THE TESTS
void activateTestCallback(const std_msgs::Int32& msg);
// UPDATE SETPOINT FOR CIRCLE TEST
void update_setpoint_for_test_circle();
// CHANGE THE GAINS
void horizontalGainCallback(const std_msgs::Int32& msg);
void verticalGainCallback(const std_msgs::Int32& msg);
void headingGainCallback(const std_msgs::Int32& msg);

// ******************************************************************************* //