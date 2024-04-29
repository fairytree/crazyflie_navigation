//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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
//    Remote Controller
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
#include "dfall_pkg/ViconSubscribeObjectName.h"

// Include the DFALL service types
#include "dfall_pkg/IntIntService.h"
#include "dfall_pkg/LoadYamlFromFilename.h"
#include "dfall_pkg/GetSetpointService.h"
#include "dfall_pkg/CMQuery.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/DefaultControllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>





// Namespacing the package
using namespace dfall_pkg;





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
// #define LQR_RATE_MODE   1   // (DEFAULT)
// #define LQR_ANGLE_MODE  2





//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// ******************************************************************************* //
// VARIABLES SPECIFIC TO THE REMOTE CONTROL FEATURE

// ROS Subscriber for the Vicon data of the remote
ros::Subscriber viconSubscriber;

// ROS Publisher for the Vicon data of the remote
ros::Publisher remote_xyz_rpy_publisher;

// ROS Publisher for the Setpoint for the remote controller
ros::Publisher remote_control_setpoint_publisher;

// Vicon object name of the Remote to follow
std::string viconObjectName_forRemote ("empty");
std::string default_viconObjectName_forRemote ("DFALL_REMOTE01");

// Boolean for whether the Remote's state should be published as a message
bool shouldPublishRemote_xyz_rpy = false;

// Boolean for whether the Remote's state should be display in the terminal window
bool shouldDisplayRemote_xyz_rpy = false;

// Boolean for whether the Remote control mode is active
bool isActive_remoteControlMode = false;

// The setpoint from the remote
float setpointFromRemote_roll  = 0.0;
float setpointFromRemote_pitch = 0.0;
float setpointFromRemote_yaw   = 0.0;
float setpointFromRemote_z     = 0.0;

// The z-height of the remote when "Activate" is pressed
float z_of_remote_previous_measurement = 0.0;
float z_when_remote_activated = 0.0;

// Roll and pitch limit (in degrees for angles)
float remoteControlLimit_roll_degrees  = 15.0;
float remoteControlLimit_pitch_degrees = 15.0;
float remoteControlLimit_roll          = 0.262;
float remoteControlLimit_pitch         = 0.262;

// Factor by which to reduce the remote control input
float remoteConrtolSetpointFactor_roll   = 1.0;
float remoteConrtolSetpointFactor_pitch  = 1.0;
float remoteConrtolSetpointFactor_yaw    = 1.0;
float remoteConrtolSetpointFactor_z      = 1.0;

// LQR Gain matrix for tracking the angle commands from the Crazyflie
std::vector<float> gainMatrixRollRate_forRemoteControl   = {5.00, 0.00, 0.00};
std::vector<float> gainMatrixPitchRate_forRemoteControl  = {0.00, 5.00, 0.00};
std::vector<float> gainMatrixYawRate_forRemoteControl    = {0.00, 0.00, 2.80};

// ******************************************************************************* //



// VARIABLES FOR SOME "ALMOST CONSTANTS"
// > Mass of the Crazyflie quad-rotor, in [grams]
float cf_mass = 25.0;
// > Coefficients of the 16-bit command to thrust conversion
std::vector<float> motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};
// The weight of the Crazyflie in [Newtons], i.e., mg
float gravity_force = cf_mass * 9.81 / 1000.0;;
// One quarter of the "gravity_force"
float gravity_force_quarter = 0.25 * gravity_force;




// VARIABLES FOR THE CONTROLLER

// Frequency at which the controller is running
float yaml_vicon_frequency = 200.0;

// Frequency at which the controller is running
float control_frequency = 200.0;


// > The setpoints for (x,y,z) position and yaw angle, in that order
float setpoint[4] = {0.0,0.0,0.4,0.0};




// The controller type to run in the "calculateControlOutput" function
int controller_mode = CONTROLLER_MODE_LQR_RATE;

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
std::vector<float> gainMatrixThrust_NineStateVector = {0.00, 0.00, 0.98, 0.00, 0.00, 0.25, 0.00, 0.00, 0.00};
std::vector<float> gainMatrixRollRate               = {0.00,-1.80, 0.00, 0.00,-1.38, 0.00, 5.20, 0.00, 0.00};
std::vector<float> gainMatrixPitchRate              = {1.80, 0.00, 0.00, 1.38, 0.00, 0.00, 0.00, 5.20, 0.00};
std::vector<float> gainMatrixYawRate                = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.30};

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE"
std::vector<float> gainMatrixThrust_SixStateVector = {0.00, 0.00, 0.98, 0.00, 0.00, 0.25};
std::vector<float> gainMatrixRollAngle             = {0.00,-0.20, 0.00, 0.00,-0.20, 0.00};
std::vector<float> gainMatrixPitchAngle            = {0.20, 0.00, 0.00, 0.20, 0.00, 0.00};

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED"
std::vector<float> gainMatrixThrust_SixStateVector_50Hz = {0.00, 0.00, 0.82, 0.00, 0.00, 0.22};
std::vector<float> gainMatrixRollAngle_50Hz             = {0.00,-0.31, 0.00, 0.00,-0.25, 0.00};
std::vector<float> gainMatrixPitchAngle_50Hz            = {0.31, 0.00, 0.00, 0.25, 0.00, 0.00};

std::vector<float> gainMatrixRollRate_Nested            = {4.00, 0.00, 0.00};
std::vector<float> gainMatrixPitchRate_Nested           = {0.00, 4.00, 0.00};
std::vector<float> gainMatrixYawRate_Nested             = {0.00, 0.00, 2.30};

int   lqr_angleRateNested_counter = 4;
float lqr_angleRateNested_prev_thrustAdjustment = 0.0;
float lqr_angleRateNested_prev_rollAngle        = 0.0;
float lqr_angleRateNested_prev_pitchAngle       = 0.0;
float lqr_angleRateNested_prev_yawAngle         = 0.0;


// The 16-bit command limits
float cmd_sixteenbit_min = 1000;
float cmd_sixteenbit_max = 60000;


// VARIABLES FOR THE ESTIMATOR

// Frequency at which the controller is running
float m_estimator_frequency = 200.0;

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
std::vector<float> PMKF_Ahat_row1_for_positions = {  0.6723, 0.0034};
std::vector<float> PMKF_Ahat_row2_for_positions = {-12.9648, 0.9352};
std::vector<float> PMKF_Kinf_for_positions      = {  0.3277,12.9648};
// > For the (roll,pitch,yaw) angles
std::vector<float> PMKF_Ahat_row1_for_angles    = {  0.6954, 0.0035};
std::vector<float> PMKF_Ahat_row2_for_angles    = {-11.0342, 0.9448};
std::vector<float> PMKF_Kinf_for_angles         = {  0.3046,11.0342};



// VARIABLES FOR THE NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;


// ROS PUBLISHER FOR SENDING OUT THE DEBUG MESSAGES
ros::Publisher debugPublisher;


// VARIABLES RELATING TO PERFORMING THE CONVERSION INTO BODY FRAME

// Boolean whether to execute the convert into body frame function
bool shouldPerformConvertIntoBodyFrame = false;


// VARIABLES RELATING TO THE PUBLISHING OF A DEBUG MESSAGE

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool shouldDisplayDebugInfo = false;


// ------------------------------------------------------
// VARIABLES THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

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
void setpointCallback(const Setpoint& newSetpoint);




// LOADING OF YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyRemoteControllerYamlCallback(const IntWithHeader & msg);
void fetchRemoteControllerYamlParameters(ros::NodeHandle& nodeHandle);




// ******************************************************************************* //
// FUNCTIONS SPECIFIC TO THE REMOTE CONTROL FEATURE

void calculateControlOutput_forRemoteControl(float stateInertial[12], Controller::Request &request, Controller::Response &response);

// CALLBACK WITH INFOMATION ABOUT WHICH REMOTE TO SUBSCRIBE TO
void viconSubscribeObjectNameCallback(const ViconSubscribeObjectName& msg);

// VICON DATA CALLBACK
void viconCallback(const ViconData& viconData);

// ACTIVATE REMOTE CONTROL MODE MESSAGE CALLBACK
void shouldActivateCallback(const std_msgs::Int32& msg);

// ******************************************************************************* //