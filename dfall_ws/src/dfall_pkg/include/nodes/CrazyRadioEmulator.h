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
//    A node to emulator a CrazyRadio node
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include <stdlib.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/FlyingVehicleState.h"

// Include the DFALL service types
#include "dfall_pkg/IntIntService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"
#include "classes/QuadrotorSimulator.h"

// SPECIFY THE PACKAGE NAMESPACE
using namespace dfall_pkg;
//using namespace std;



//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------





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



// The status of the Radio connection
int m_radio_state = CRAZY_RADIO_STATE_DISCONNECTED;

// Timer for the battery voltage updates
ros::Timer m_timer_battery_voltage_updates;

// Timer for the state estimte updates
ros::Timer m_timer_state_estimate_update;


// PUBLISHERS

// Publisher for the status of the radio connection
ros::Publisher crazyRadioStatusPublisher;

// Publisher for sending a "Flying Agent Client Command"
ros::Publisher flyingAgentClientCommandPublisher;

// Publisher for the raw battery voltage
ros::Publisher batteryVoltagePublisher;

// Publisher for the onboard state estimate
ros::Publisher  cfStateEstimatorPublisher;

// Publisher for the control commands
// > Note this is not needed for the real CrazyRadio node because
//   this command goes out over the radio, but for emulation we publish
//   the control command
ros::Publisher controlCommandPublisher;

// Publisher for the simulation state
// > This is used by the MocapEmulator node
ros::Publisher  cfSimulationStatePublisher;





// VARIABLES THAT ARE LOADED FROM THE YAML FILE

// Frequency of requesting the battery voltage, in [seconds]
float yaml_battery_polling_period_in_seconds = 0.2f;

// Flag for whether to use height as a trigger for
// publishing a motors-OFF command
bool yaml_isEnabled_strictSafety = true;
float yaml_maxHeight_for_strictSafety_meters = 1.2;

// // Battery thresholds while in the "motors off" state, in [Volts]
// float yaml_battery_voltage_threshold_lower_while_standby = 3.30f;
// float yaml_battery_voltage_threshold_upper_while_standby = 4.20f;

// // Battery thresholds while in the "flying" state, in [Volts]
// float yaml_battery_voltage_threshold_lower_while_flying = 2.60f;
// float yaml_battery_voltage_threshold_upper_while_flying = 3.70f;

// // Delay before changing the state of the battery, in [number of measurements]
// // > Note that the delay in seconds therefore depends on the polling period
// int yaml_battery_delay_threshold_to_change_state = 5;





// VARIABLES FOR SIMULATING A QUAROTOR

// A QUADROTORS SIMULATOR
QuadrotorSimulator m_quadrotor_sim;

// SIMULATION FREQUENCY FOR EMULATING A CRAZYFLIE
//float yaml_cfStateEstimate_polling_period_in_seconds = 0.02f;
float yaml_cfSimulation_frequency = 200.0;
float yaml_cfSimulation_deltaT_in_seconds = 0.005;

// HOW OFTEN TO PUBLISH THE SIMULATION STATE AS AN
// ONBOARD ESTIMATE
// > i.e., this integer divided by the
//   "cfSimulation_frequency" gives the simulation
//    equivalent of "cfStateEstimate_polling_period"
int yaml_cfSimulation_stateEstimate_sendEvery = 4;

// THE COEFFICIENT OF THE 16-BIT COMMAND TO THRUST CONVERSION
std::vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};

// THE MIN AND MAX FOR SATURATING THE 16-BIT THRUST COMMANDS
float yaml_command_sixteenbit_min = 1000.0f;
float yaml_command_sixteenbit_max = 65000.0f;





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

// CHANGE THE STATE OF THE RADIO
void change_radio_state_to( int new_state );

// PUBLISH THE CURRENT STATE OF THE RADIO CONNECTION
void publishCurrentRadioState();

// PERFORM CONNECT AND DISCONNECT
void connect();
void disconnect();

// RESPOND TO CONNECTED AND DISCONNECTED STATUS CHANGE
void connected_callback(const ros::TimerEvent&);
void connection_failed();
void connection_lost();
void disconnected_callback(const ros::TimerEvent&);


// SUBSCRIBER CALLBACKS
void crazyRadioCommandCallback(const IntWithHeader & msg);
void controlCommandCallback(const ControlCommand & msg);

// SERVICE CALLBACK
bool getCurrentCrazyRadioStatusServiceCallback(IntIntService::Request &request, IntIntService::Response &response);


// UPDATE BATTERY VOLTAGE TIMER CALLBACK
void timerCallback_update_battery_voltage(const ros::TimerEvent&);

// UPDATE STATE ESTIMATE TIMER CALLBACK
void timerCallback_update_cfStateEstimate(const ros::TimerEvent&);

// LOAD YAML PARAMETER FUNCTIONS
void isReadyBatteryMonitorYamlCallback(const IntWithHeader & msg);
void fetchBatteryMonitorYamlParameters(ros::NodeHandle& nodeHandle);
void fetchCrazyRadioConfigYamlParameters(ros::NodeHandle& nodeHandle);