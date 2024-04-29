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
//    Emulator for the Motion Capture data, and simulates a fleet of quadrotor
//    to prouce the emulated data
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include "ros/ros.h"
#include <stdlib.h>
#include <std_msgs/String.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/FlyingVehicleState.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/CrazyflieContext.h"
#include "dfall_pkg/AreaBounds.h"
#include "dfall_pkg/Setpoint.h"

// Include the DFALL service types
#include "dfall_pkg/CMQuery.h"
#include "dfall_pkg/CMQueryCrazyflieName.h"
//#include "dfall_pkg/IntIntService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"
#include "classes/QuadrotorSimulator.h"

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









//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// TIMER FOR THE MOTION CAPTURE PUBLISHING
ros::Timer m_timer_mocap_publisher;

// FREQUENCY OF THE MOTION CAPTURE EMULATOR [Hertz]
float yaml_mocap_frequency = 200.0;
float yaml_mocap_deltaT_in_seconds = 0.005;

// THE COEFFICIENT OF THE 16-BIT COMMAND TO THRUST CONVERSION
std::vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};

// THE MIN AND MAX FOR SATURATING THE 16-BIT THRUST COMMANDS
float yaml_command_sixteenbit_min = 1000;
float yaml_command_sixteenbit_max = 65000;

// A VECTOR OF QUADROTORS
std::vector<QuadrotorSimulator> m_quadrotor_fleet;

// A VECTOR OF FLYING VEHICLE STATES
std::vector<FlyingVehicleState> m_flying_fleet_states;
std::vector<ros::Subscriber> m_flying_fleet_state_subscribers;
std::vector<AreaBounds> m_flying_fleet_areaBounds;

// PUBLISHER FOR THE MOTION CAPTURE DATA
ros::Publisher m_mocapDataPublisher;

// SERVICE CLIENT FOR THE CENTRAL MANAGER
ros::ServiceClient m_centralManagerService;


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

// CALLBACK FOR EVERYTIME MOCAP DATA SHOULD BE PUBLISHED
void timerCallback_mocap_publisher(const ros::TimerEvent&);

// CALLBACK FOR RECEIVEING CRAZYFLIE SIMULATION DATA
// > This data is published by the CrazyRadioEmulator node
void cfSimulationStateCallback(const FlyingVehicleState & msg);

// FUNCTIONS FOR THE CONTEXT OF THIS AGENT
// > Callback that the context database changed
void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);
// > For loading the context of each quadrotor simulator
void loadContextForEachQuadrotor();

// FOR LOADING THE YAML PARAMETERS
void fetchMocapEmulatorConfigYamlParameters();

