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
//    A controller to test the motors
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/TestMotorsControllerService.h"






//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//     I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//     I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//     I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
//    III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
//    ----------------------------------------------------------------------------------





//    ------------------------------------------------------------------------------
//     OOO   U   U  TTTTT  EEEEE  RRRR 
//    O   O  U   U    T    E      R   R
//    O   O  U   U    T    EEE    RRRR
//    O   O  U   U    T    E      R  R
//     OOO    UUU     T    EEEEE  R   R
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

// This function is the callback that is linked to the "TestMotorsController"
// service that is advertised in the main function. This must have arguments
// that match the "input-output" behaviour defined in the "Controller.srv"
// file (located in the "srv" folder)
//
// The arument "request" is a structure provided to this service with the
// following two properties:
//
// >> request.ownCrazyflie
// This property is itself a structure of type "FlyingVehicleState",  which is
// defined in the file "FlyingVehicleState.msg", and has the following properties
//     string vehicleName
//     float64 x                         The x position of the Crazyflie [metres]
//     float64 y                         The y position of the Crazyflie [metres]
//     float64 z                         The z position of the Crazyflie [metres]
//     float64 roll                      The roll component of the intrinsic Euler angles [radians]
//     float64 pitch                     The pitch component of the intrinsic Euler angles [radians]
//     float64 yaw                       The yaw component of the intrinsic Euler angles [radians]
//     float64 acquiringTime #delta t    The time elapsed since the previous "FlyingVehicleState" was received [seconds]
//     bool isValid                      A boolean indicted whether the Crazyflie for visible at the time of this measurement
// The values in these properties are directly the measurement taken by the
// motion capture system of the Crazyflie that is to be controlled by this
// service.
//
// >> request.otherCrazyflies
// This property is an array of "FlyingVehicleState" structures, what allows access
// to the motion capture measurements of other Crazyflies.
//
// The argument "response" is a structure that is expected to be filled in by
// this service by this function, it has only the following property
//
// >> response.ControlCommand
// This property is iteself a structure of type "ControlCommand", which is
// defined in the file "ControlCommand.msg", and has the following properties:
//     uint16 motorCmd1                 The command sent to the Crazyflie for motor 1
//     uint16 motorCmd2                 ... same for motor 2
//     uint16 motorCmd3                 ... same for motor 3
//     uint16 motorCmd4                 ... same for motor 4
//     uint8 xControllerMode            The mode sent to the Crazyflie for what controller to run for the body frame x-axis
//     uint8 yControllerMode            ... same body frame y-axis
//     uint8 zControllerMode            ... same body frame z-axis
//     uint8 yawControllerMode          ... same body frame yaw
//     float32 xControllerSetpoint      The setpoint sent to the Crazyflie for the body frame x-axis controller
//     float32 yControllerSetpoint      ... same body frame y-axis
//     float32 zControllerSetpoint      ... same body frame z-axis
//     float32 yawControllerSetpoint    ... same body frame yaw
// 
// IMPORTANT NOTES FOR "{x,y,z,yaw}ControllerMode"  AND AXIS CONVENTIONS
// > The allowed values for "{x,y,z,yaw}ControllerMode" are in the
//   "Constants.h" header file, they are:
//   - CF_ONBOARD_CONTROLLER_MODE_OFF
//   - CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE
//   - CF_ONBOARD_CONTROLLER_MODE_ANGLE
//   - CF_ONBOARD_CONTROLLER_MODE_VELOCITY
//   - CF_ONBOARD_CONTROLLER_MODE_POSITION
//
// > The most common option to use for the {x,y,yaw} controller is
//   the CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE option.
//
// > The most common option to use for the {z} controller is
//   the CF_ONBOARD_CONTROLLER_MODE_OFF option, and thus the
//   body frame z-axis is controlled by the motorCmd{1,2,3,4}
//   values that you set.
//
// > When the CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE is selected, then:
//   1) the ".xControllerSetpoint", ".yControllerSetpoint", and
//      ".yawControllerSetpoint" properties of "response.ControlCommand"
//      specify the angular rate in [radians/second] that will be requested
//      from the PID controllers running in the Crazyflie 2.0 firmware.
//   2) the axis convention for the roll, pitch, and yaw body rates,
//      i.e., as set in the {y,x,yaw}ControllerSetpoint properties of
//      the "response.ControlCommand" that you return, is a right-hand
//      coordinate axes with x-forward and z-upwards (i.e., the positive
//      z-axis is aligned with the direction of positive thrust). To
//      assist, the ASCII art below depicts this convention.
//   3) the ".motorCmd1" to ".motorCmd4" properties of
//      "response.ControlCommand" are the baseline motor commands
//      requested from the Crazyflie, with the adjustment for body rates
//      being added on top of this in the firmware (i.e., as per the
//      code of the "distribute_power" found in the firmware).
//
// ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
//
//  > This is a top view,
//  > M1 to M4 stand for Motor 1 to Motor 4,
//  > "CW"  indicates that the motor rotates Clockwise,
//  > "CCW" indicates that the motor rotates Counter-Clockwise,
//  > By right-hand axis convention, the positive z-direction points out
//    of the screen,
//  > This being a "top view" means that the direction of positive thrust
//    produced by the propellers is also out of the screen.
//
//        ____                         ____
//       /    \                       /    \
//  (CW) | M4 |           x           | M1 | (CCW)
//       \____/\          ^          /\____/
//            \ \         |         / /
//             \ \        |        / /
//              \ \______ | ______/ /
//               \        |        /
//                |       |       |
//        y <-------------o       |
//                |               |
//               / _______________ \
//              / /               \ \
//             / /                 \ \
//        ____/ /                   \ \____
//       /    \/                     \/    \
// (CCW) | M3 |                       | M2 | (CW)
//       \____/                       \____/
//
//
//
//
bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner

	// Initialise a static variable for counting time "ticks"
	static int ticks = 0;

	// Increment the time counter every time this function is called
	ticks++;

	// Specify that all controllers are disabled
	response.controlOutput.xControllerMode   = CF_ONBOARD_CONTROLLER_MODE_OFF;
	response.controlOutput.yControllerMode   = CF_ONBOARD_CONTROLLER_MODE_OFF;
	response.controlOutput.zControllerMode   = CF_ONBOARD_CONTROLLER_MODE_OFF;
	response.controlOutput.yawControllerMode = CF_ONBOARD_CONTROLLER_MODE_OFF;

	// Fill in zero for the controller setpoints
	response.controlOutput.xControllerSetpoint   = 0.0;
	response.controlOutput.yControllerSetpoint   = 0.0;
	response.controlOutput.zControllerSetpoint   = 0.0;
	response.controlOutput.yawControllerSetpoint = 0.0;

	// Fill in all motor thrusts as zero
	response.controlOutput.motorCmd1 = 0;
	response.controlOutput.motorCmd2 = 0;
	response.controlOutput.motorCmd3 = 0;
	response.controlOutput.motorCmd4 = 0;


	if (ticks < 300)
		response.controlOutput.motorCmd1 = 9000;
	else if (ticks < 600)
		response.controlOutput.motorCmd2 = 9000;
	else if (ticks < 900)
		response.controlOutput.motorCmd3 = 9000;
	else if (ticks < 1200)
		response.controlOutput.motorCmd4 = 9000;
	else
	{
		response.controlOutput.motorCmd4 = 9000;
		ticks = 0;
	}



	// Return "true" to indicate that the control computation was
	// performed successfully
	return true;
}







//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------


// This function does NOT need to be edited 
int main(int argc, char* argv[]) {

	// Starting the ROS-node
	ros::init(argc, argv, "TestMotorsControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "TestMotorsControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[TEST MOTORS CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "FlyingAgentClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "FlyingAgentClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[TEST MOTORS CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[TEST MOTORS CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



    // PUBLISHERS AND SUBSCRIBERS

    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "TestMotorsController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("TestMotorsController", calculateControlOutput);



    // Print out some information to the user.
    ROS_INFO("[TEST MOTORS CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}