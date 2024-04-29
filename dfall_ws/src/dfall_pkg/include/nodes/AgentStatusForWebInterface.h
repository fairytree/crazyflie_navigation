//    Copyright (C) 2020, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE), Paul Beuchat
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
//    The service that provides data to the web interface
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
//#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>

// Include the standard message types
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/SetpointWithHeader.h"
#include "dfall_pkg/FlyingVehicleState.h"
#include "dfall_pkg/DebugMsg.h"

// Include the DFALL service types
#include "dfall_pkg/IntStringService.h"

// Include the shared definitions
#include "nodes/Constants.h"
//#include "classes/GetParamtersAndNamespaces.h"

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


// The crazyradio status
// > as received via messages
int m_crazyradio_status = CRAZY_RADIO_STATE_DISCONNECTED;

// The battery level
// > as received via messages
int m_battery_level = BATTERY_LEVEL_UNAVAILABLE;

// The flying state of the agent
// > as received via messages
int m_agent_operating_state = STATE_UNAVAILABLE;

// The instant controller,
// > as received via messages
int m_instant_controller = DEFAULT_CONTROLLER;

// The setpoint of the default controller
// > as received via messages
float m_setpoint_default[4] = {0.0,0.0,0.4,0.0};

// The setpoint of the student controller
// > as received via messages
float m_setpoint_student[4] = {0.0,0.0,0.4,0.0};

// The debug values of the student controller
// > as received via messages
float m_debug_values_student[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// The current state estimate of the Crazyflie
// > as received via messages
float m_cf_state_estimate_xyz_rpy[6] = {0.0,0.0,0.0,0.0,0.0,0.0};





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


// Callbacks for messages with the status of things:
// > For the status of the crazyradio
void crazyRadioStatusCallback(const std_msgs::Int32& msg);
// > For the battery level
void newBatteryLevelCallback(const std_msgs::Int32& msg);
// > For the flying state of the agent
void agentOperatingStateCallback(const std_msgs::Int32& msg);
// > For the instant controller
void instantControllerChangedCallback(const std_msgs::Int32& msg);
// > For the Default Controller Setpoint
void defaultControllerSetpointChangedCallback(const SetpointWithHeader& newSetpoint);
// > For the Student Controller Setpoint
void studentControllerSetpointChangedCallback(const SetpointWithHeader& newSetpoint);
// > For the Student Controller Debug Values
void studentControllerDebugValuesCallback(const DebugMsg& newDebugMsg);
// > For the State Estimate from the Crazyflie
void cfStateEstimateCallback(const FlyingVehicleState & newStateEstimate);

// Service callback for providing status to the Web Interface
bool statusForWebInterfaceCallback(IntStringService::Request &request, IntStringService::Response &response);