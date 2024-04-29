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
//    A CS1 Controller for students build from
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
#include "dfall_pkg/IntIntService.h"
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"

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

// NOTE: many constants are already defined in the
//       "Constant.h" header file





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



// VARAIBLES FOR VALUES LOADED FROM THE YAML FILE
// > the mass of the crazyflie, in [grams]
float yaml_cf_mass_in_grams = 30.0;

// > the frequency at which the controller is running
float yaml_control_frequency = 200.0;

// > the coefficients of the 16-bit command to thrust conversion
//std::vector<float> yaml_motorPoly(3);
std::vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};


// The min and max for saturating 16 bit thrust commands
float yaml_command_sixteenbit_min = 1000;
float yaml_command_sixteenbit_max = 60000;

// > the default setpoint, the ordering is (x,y,z,yaw),
//   with units [meters,meters,meters,radians]
std::vector<float> yaml_default_setpoint = {0.0,0.0,0.4,0.0};

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool yaml_shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool yaml_shouldDisplayDebugInfo = false;

// The LQR Controller parameters for "LQR_RATE_MODE"
std::vector<float> yaml_gainMatrixThrust_NineStateVector  =  { 0.00, 0.00, 0.98, 0.00, 0.00, 0.25, 0.00, 0.00, 0.00};
std::vector<float> yaml_gainMatrixRollRate                =  { 0.00,-6.20, 0.00, 0.00,-3.00, 0.00, 5.20, 0.00, 0.00};
std::vector<float> yaml_gainMatrixPitchRate               =  { 6.20, 0.00, 0.00, 3.00, 0.00, 0.00, 0.00, 5.20, 0.00};
std::vector<float> yaml_gainMatrixYawRate                 =  { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.30};

// Integrator gains
float yaml_integratorGain_forThrust = 0.0;
float yaml_integratorGain_forTauXY  = 0.0;
float yaml_integratorGain_forTauYaw = 0.0;

// The state of the integrator
int m_integratorState = INTEGRATOR_FLAG_OFF;

// The current value of the integrator for each axis
float m_newtons_intergal = 0.0;
float m_tau_x_integral   = 0.0;
float m_tau_y_integral   = 0.0;
float m_tau_z_integral   = 0.0;



// The weight of the Crazyflie in Newtons, i.e., mg
float m_cf_weight_in_newtons = 0.0;

// The state space matrices of lead compensator controller
// the following corresponds to k=0.2, T=1.5, alpha=0.1
// this works well
// float m_A=0.967216;
// float m_B=0.00491758;
// float m_C=-12.0;
// float m_D=2.0;

// the following corresponds to k=0.1, T=1.0, alpha=0.5
// this is barely stabilizing
float m_A = 0.99004983;
float m_B = 0.00497508;
float m_C = -0.2;
float m_D = 0.2;

// The state of lead compensator controller
float m_controller_state = 0.0;

// The state space matrices of lag compensator controller
// the following corresponds to T = 0.0, alpha = 1.0
// (i.e., lag controller inactive)
float m_lag_A = 0.0;
float m_lag_B = 0.0;
float m_lag_C = 0.0;
float m_lag_D = 1.0;

// The state of lag compensator controller
float m_lag_controller_state = 0.0;

// The inertial x measurement during a time window of 300 measurements
std::vector<float>  m_x_inertial_buffer (300,0);

// The time delay as an integer of how many measurements to delay by
int m_time_delay_in_steps = 0;

// The pitch measurement error in [radians]
float m_pitch_error_in_rad = 0;

// The location error of the Crazyflie at the "previous" time step
float m_previous_stateErrorInertial[9];

// The setpoint to be tracked, the ordering is (x,y,z,yaw),
// with units [meters,meters,meters,radians]
std::vector<float>  m_setpoint{0.0,0.0,0.4,0.0};




// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;

// ROS Publisher for informing the network about
// changes to the setpoint
ros::Publisher m_setpointChangedPublisher;

// ROS Publisher for informing the network about
// changes to the integrator state
ros::Publisher m_integratorStateChangedPublisher;





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

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// REQUEST INTEGRATOR STATE CHANGE CALLBACK
void requestIntegratorStateChangeCallback(const IntWithHeader& msg);

// CHANGE INTEGRATOR STATE FUNCTION
void setNewIntegratorState(int newIntegratorState);

// GET CURRENT INTEGRATOR STATE SERVICE CALLBACK
bool getCurrentIntegratorStateCallback(IntIntService::Request &request, IntIntService::Response &response);

// REQUEST TIME DELAY CALLBACK
void requestTimeDelayChangeCallback(const IntWithHeader& msg);

// CHANGE TIME DELAY FUNCTION
void setNewTimeDelay(int newTimeDelay);

// REQUEST PITCH ERROR CALLBACK
void requestPitchErrorChangeCallback(const SetpointWithHeader& newSetpoint);

// REQUEST CONTROLLER PARAMETERS CALLBACK
void requestControllerParametersChangeCallback(const SetpointWithHeader& newSetpoint);

// REQUEST LAG CONTROLLER PARAMETERS CALLBACK
void requestLagControllerParametersChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void convertIntoDiscreteTimeParameters(float k, float T, float alpha);

// CHANGE LAG CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void convertLagIntoDiscreteTimeParameters(float T, float alpha);

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived);

// FOR LOADING THE YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyCsoneControllerYamlCallback(const IntWithHeader & msg);
void fetchCsoneControllerYamlParameters(ros::NodeHandle& nodeHandle);