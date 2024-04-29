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
//    A Csone Controller for students build from
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/CsoneControllerService.h"






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

// This function is the callback that is linked to the "CsoneController"
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

	// Initialise static variables for handling the circular buffer
	// of inertial x measurement
	static int write_index = 0;
	static int read_index = 0;

	// Increment the write index
	write_index += 1;
	// And wrap it back into range if necessary
	if (write_index>=m_x_inertial_buffer.size())
	{
		write_index = 0;
	}

	// Compute the next read index based on the delay
	read_index = write_index - m_time_delay_in_steps;
	// And wrap it back into range if necessary
	if (read_index<0)
	{
		read_index += m_x_inertial_buffer.size();
	}

	// Write the new data to the buffer
	m_x_inertial_buffer[write_index] = request.ownCrazyflie.x;

	// Read the data for this time step from the buffer
	float inertial_x_for_this_time_step = m_x_inertial_buffer[read_index];


	// Define a local array to fill in with the state error
	float stateErrorInertial[9];

	// Fill in the (x,y,z) position error
	stateErrorInertial[0] = inertial_x_for_this_time_step - m_setpoint[0];
	stateErrorInertial[1] = request.ownCrazyflie.y - m_setpoint[1];
	stateErrorInertial[2] = request.ownCrazyflie.z - m_setpoint[2];

	// Compute an estimate of the velocity
	// > But only if this is NOT the first call to the controller
	if (!request.isFirstControllerCall)
	{
		// > Compute as simply the derivative between the current and previous position
		stateErrorInertial[3] = (stateErrorInertial[0] - m_previous_stateErrorInertial[0]) * yaml_control_frequency;
		stateErrorInertial[4] = (stateErrorInertial[1] - m_previous_stateErrorInertial[1]) * yaml_control_frequency;
		stateErrorInertial[5] = (stateErrorInertial[2] - m_previous_stateErrorInertial[2]) * yaml_control_frequency;
	}
	else
	{
		// Set the velocities to zero
		stateErrorInertial[3] = 0.0;
		stateErrorInertial[4] = 0.0;
		stateErrorInertial[5] = 0.0;
	}
	
	// Fill in the roll and pitch angle measurements directly
	stateErrorInertial[6] = request.ownCrazyflie.roll;
	stateErrorInertial[7] = request.ownCrazyflie.pitch;

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = request.ownCrazyflie.yaw - m_setpoint[3];
	// > Second, "unwrap" the yaw error to the interval ( -pi , pi )
	while(yawError > PI) {yawError -= 2 * PI;}
	while(yawError < -PI) {yawError += 2 * PI;}
	// > Third, put the "yawError" into the "stateError" variable
	stateErrorInertial[8] = yawError;


	// CONVERSION INTO BODY FRAME
	// Conver the state erorr from the Inertial frame into the Body frame
	// > Note: the function "convertIntoBodyFrame" is implemented in this file
	//   and by default does not perform any conversion. The equations to convert
	//   the state error into the body frame should be implemented in that function
	//   for successful completion of the classroom exercise
	float stateErrorBody[9];
	convertIntoBodyFrame(stateErrorInertial, stateErrorBody, request.ownCrazyflie.yaw);


	// SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
	// > as we have already used previous error we can now update it update it
	for(int i = 0; i < 9; ++i)
	{
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}


	
	// PERFORM THE "u=-Kx" CONTROLLER COMPUTATIONS

	// Instantiate local variables for the responses
	float thrustAdjustment = 0;
	float rollRate_forResponse = 0;
	float pitchRate_forResponse = 0;
	float yawRate_forResponse = 0;

	// Perform the "-Kx" LQR computation for the yaw rate to respoond with
	for(int i = 0; i < 9; ++i)
	{
		// For the z-controller
		thrustAdjustment -= yaml_gainMatrixThrust_NineStateVector[i] * stateErrorBody[i];
		// // For the x-controller
		// pitchRate_forResponse -= yaml_gainMatrixPitchRate[i] * stateErrorBody[i];
		// For the y-controller
		rollRate_forResponse -= yaml_gainMatrixRollRate[i] * stateErrorBody[i];
		// For the yaw-controller
		yawRate_forResponse -= yaml_gainMatrixYawRate[i] * stateErrorBody[i];
	}



	// For the x-controller:
	// > Execute the discrete-time state space equivalent of the lead-lag
	//   compensator specified.
	// > NOTE: the new pitch reference, i.e., the output to the controller,
	//		   MUST be calucated before the state update equation
	//         is evaulated, in equations:
	//         FIRST:    u_ld(t)   = C_ld x_ld(t) + D_ld e(t)
	//		   			 y(t)      = C_lg x_lg(t) + D_lg u_ld(t)
	//         SECOND:   x_ld(t+1) = A_ld x_ld(t) + B_ld e(t)
	//					 x_lg(t+1) = A_lg x_lg(t) + B_lg u_ld(t)

	// > FIRST: compute the new pitch reference
	float lead_u = m_C*m_controller_state - m_D*stateErrorBody[0];
	float pitch_ref = m_lag_C*m_lag_controller_state + m_lag_D*lead_u;

	// > SECOND: evaluate the state update equation
	m_controller_state = m_A*m_controller_state - m_B*stateErrorBody[0];
	m_lag_controller_state = m_lag_A*m_lag_controller_state + m_lag_B*lead_u;

	// > THIRD: perform the inner controller
	// Subtract the user input pitch measurement error from the measurement to recover the "true" pitch
	pitchRate_forResponse = 5.0*(pitch_ref-(request.ownCrazyflie.pitch-m_pitch_error_in_rad));



	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"
	// > i.e., put the computed body rate commands into the "response" variable

	// Specify the mode of each controller
	response.controlOutput.xControllerMode   = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
	response.controlOutput.yControllerMode   = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
	response.controlOutput.zControllerMode   = CF_ONBOARD_CONTROLLER_MODE_OFF;
	response.controlOutput.yawControllerMode = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;

	// Put the computed rates into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.xControllerSetpoint   = pitchRate_forResponse;
	response.controlOutput.yControllerSetpoint   = rollRate_forResponse;
	response.controlOutput.zControllerSetpoint   = 0.0;
	response.controlOutput.yawControllerSetpoint = yawRate_forResponse;



	// PERFORM THE INTEGRATOR COMPUTATIONS
	if (m_integratorState == INTEGRATOR_FLAG_ON)
	{
		m_newtons_intergal   -= yaml_integratorGain_forThrust * stateErrorBody[2] / yaml_control_frequency;
		m_tau_x_integral     += yaml_integratorGain_forTauXY  * stateErrorBody[1] / yaml_control_frequency;
		m_tau_y_integral     -= yaml_integratorGain_forTauXY  * stateErrorBody[0] / yaml_control_frequency;
		m_tau_z_integral     -= yaml_integratorGain_forTauYaw * stateErrorBody[8] / yaml_control_frequency;
	}

	 float cmd1_integral_adjustments = m_newtons_intergal - m_tau_x_integral - m_tau_y_integral - m_tau_z_integral;
	 float cmd2_integral_adjustments = m_newtons_intergal - m_tau_x_integral + m_tau_y_integral + m_tau_z_integral;
	 float cmd3_integral_adjustments = m_newtons_intergal + m_tau_x_integral + m_tau_y_integral - m_tau_z_integral;
	 float cmd4_integral_adjustments = m_newtons_intergal + m_tau_x_integral - m_tau_y_integral + m_tau_z_integral;

	// Put the thrust commands into the "response" variable.
	// The thrust adjustment computed by the controller need to be added to the
	// the feed-forward thrust that "counter-acts" gravity.
	// > NOTE: remember that the thrust is commanded per motor, hence divide
	//         the thrusts by 4.
	// > NOTE: the "m_cf_weight_in_newtons" value is the total thrust required
	//         as feed-forward. Assuming the the Crazyflie is symmetric, this
	//         value is divided by four.
	float feed_forward_thrust_per_motor = m_cf_weight_in_newtons / 4.0;
	float thrust_request_per_motor = thrustAdjustment / 4.0 + feed_forward_thrust_per_motor;
	// > NOTE: the function "computeMotorPolyBackward" converts the input argument
	//         from Newtons to the 16-bit command expected by the Crazyflie.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor + cmd1_integral_adjustments);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor + cmd2_integral_adjustments);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor + cmd3_integral_adjustments);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor + cmd4_integral_adjustments);



	if (yaml_shouldPublishDebugMessage)
	{
		//  ***********************************************************
		//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
		//  D   D  E      B   B  U   U  G           MM MM  S      G
		//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
		//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
		//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG

		// DEBUGGING CODE:
		// As part of the D-FaLL-System we have defined a message type names"DebugMsg".
		// By fill this message with data and publishing it you can display the data in
		// real time using rpt plots. Instructions for using rqt plots can be found on
		// the wiki of the D-FaLL-System repository

		// Instantiate a local variable of type "DebugMsg", see the file "DebugMsg.msg"
		// (located in the "msg" folder) to see the full structure of this message.
		DebugMsg debugMsg;

		// Fill the debugging message with the data provided by Vicon
		debugMsg.vicon_x = request.ownCrazyflie.x;
		debugMsg.vicon_y = request.ownCrazyflie.y;
		debugMsg.vicon_z = request.ownCrazyflie.z;
		debugMsg.vicon_roll = request.ownCrazyflie.roll;
		debugMsg.vicon_pitch = request.ownCrazyflie.pitch;
		debugMsg.vicon_yaw = request.ownCrazyflie.yaw;

		// Fill in the debugging message with any other data you would like to display
		// in real time. For example, it might be useful to display the thrust
		// adjustment computed by the z-altitude controller.
		// The "DebugMsg" type has 10 properties from "value_1" to "value_10", all of
		// type "float64" that you can fill in with data you would like to plot in
		// real-time.
		// debugMsg.value_1 = thrustAdjustment;
		// ......................
		// debugMsg.value_10 = your_variable_name;

		// Publish the "debugMsg"
		m_debugPublisher.publish(debugMsg);
	}


	if (yaml_shouldDisplayDebugInfo)
	{
		//  ***********************************************************
		//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
		//  D   D  E      B   B  U   U  G           MM MM  S      G
		//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
		//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
		//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG
		// An alternate debugging technique is to print out data directly to the
		// command line from which this node was launched.

		// An example of "printing out" the data from the "request" argument to the
		// command line. This might be useful for debugging.
		ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
		ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
		ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
		ROS_INFO_STREAM("roll: " << request.ownCrazyflie.roll);
		ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
		ROS_INFO_STREAM("yaw: " << request.ownCrazyflie.yaw);
		ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);

		// An example of "printing out" the control actions computed.
		ROS_INFO_STREAM("thrustAdjustment = " << thrustAdjustment);
		ROS_INFO_STREAM("controlOutput.xControllerSetpoint   = " << response.controlOutput.xControllerSetpoint);
		ROS_INFO_STREAM("controlOutput.yControllerSetpoint   = " << response.controlOutput.yControllerSetpoint);
		ROS_INFO_STREAM("controlOutput.yawControllerSetpoint = " << response.controlOutput.yawControllerSetpoint);

		// An example of "printing out" the per motor 16-bit command computed.
		ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

		// An example of "printing out" the "thrust-to-command" conversion parameters.
		// ROS_INFO_STREAM("motorPoly 0:" << yaml_motorPoly[0]);
		// ROS_INFO_STREAM("motorPoly 1:" << yaml_motorPoly[1]);
		// ROS_INFO_STREAM("motorPoly 2:" << yaml_motorPoly[2]);
	}

	// Return "true" to indicate that the control computation was
	// performed successfully
	return true;
}




//    ------------------------------------------------------------------------------
//    RRRR    OOO   TTTTT    A    TTTTT  EEEEE       III  N   N  TTTTT   OOO
//    R   R  O   O    T     A A     T    E            I   NN  N    T    O   O
//    RRRR   O   O    T    A   A    T    EEE          I   N N N    T    O   O
//    R  R   O   O    T    AAAAA    T    E            I   N  NN    T    O   O
//    R   R   OOO     T    A   A    T    EEEEE       III  N   N    T     OOO
//
//    BBBB    OOO   DDDD   Y   Y       FFFFF  RRRR     A    M   M  EEEEE
//    B   B  O   O  D   D   Y Y        F      R   R   A A   MM MM  E
//    BBBB   O   O  D   D    Y         FFF    RRRR   A   A  M M M  EEE
//    B   B  O   O  D   D    Y         F      R  R   AAAAA  M   M  E
//    BBBB    OOO   DDDD     Y         F      R   R  A   A  M   M  EEEEE
//    ----------------------------------------------------------------------------------

// The arguments for this function are as follows:
// stateInertial
// This is an array of length 9 with the estimates the error of of the following values
// relative to the sepcifed setpoint:
//     stateInertial[0]    x position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[1]    y position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[2]    z position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[3]    x-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[4]    y-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[5]    z-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[6]    The roll  component of the intrinsic Euler angles [radians]
//     stateInertial[7]    The pitch component of the intrinsic Euler angles [radians]
//     stateInertial[8]    The yaw   component of the intrinsic Euler angles [radians]
// 
// stateBody
// This is an empty array of length 9, this function should fill in all elements of this
// array with the same ordering as for the "stateInertial" argument, expect that the (x,y)
// position and (x,y) velocities are rotated into the body frame.
//
// yaw_measured
// This is the yaw component of the intrinsic Euler angles in [radians] as measured by
// the Vicon motion capture system
//
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
{
	float sinYaw = sin(yaw_measured);
	float cosYaw = cos(yaw_measured);

	// Fill in the (x,y,z) position estimates to be returned
	stateBody[0] = stateInertial[0] * cosYaw  +  stateInertial[1] * sinYaw;
	stateBody[1] = stateInertial[1] * cosYaw  -  stateInertial[0] * sinYaw;
	stateBody[2] = stateInertial[2];

	// Fill in the (x,y,z) velocity estimates to be returned
	stateBody[3] = stateInertial[3] * cosYaw  +  stateInertial[4] * sinYaw;
	stateBody[4] = stateInertial[4] * cosYaw  -  stateInertial[3] * sinYaw;
	stateBody[5] = stateInertial[5];

	// Fill in the (roll,pitch,yaw) estimates to be returned
	stateBody[6] = stateInertial[6];
	stateBody[7] = stateInertial[7];
	stateBody[8] = stateInertial[8];
}





//    ------------------------------------------------------------------------------
//    N   N  EEEEE  W     W   TTTTT   OOO   N   N        CCCC  M   M  DDDD
//    NN  N  E      W     W     T    O   O  NN  N       C      MM MM  D   D
//    N N N  EEE    W     W     T    O   O  N N N  -->  C      M M M  D   D
//    N  NN  E       W W W      T    O   O  N  NN       C      M   M  D   D
//    N   N  EEEEE    W W       T     OOO   N   N        CCCC  M   M  DDDD
//
//     CCCC   OOO   N   N  V   V  EEEEE  RRRR    SSSS  III   OOO   N   N
//    C      O   O  NN  N  V   V  E      R   R  S       I   O   O  NN  N
//    C      O   O  N N N  V   V  EEE    RRRR    SSS    I   O   O  N N N
//    C      O   O  N  NN   V V   E      R  R       S   I   O   O  N  NN
//     CCCC   OOO   N   N    V    EEEEE  R   R  SSSS   III   OOO   N   N
//    ----------------------------------------------------------------------------------

float computeMotorPolyBackward(float thrust)
{
	// Compute the 16-but command that would produce the requested
	// "thrust" based on the quadratic mapping that is described
	// by the coefficients in the "yaml_motorPoly" variable.
	float cmd_16bit = (-yaml_motorPoly[1] + sqrt(yaml_motorPoly[1] * yaml_motorPoly[1] - 4 * yaml_motorPoly[2] * (yaml_motorPoly[0] - thrust))) / (2 * yaml_motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd_16bit < yaml_command_sixteenbit_min)
	{
		cmd_16bit = 0;
	}
	else if (cmd_16bit > yaml_command_sixteenbit_max)
	{
		cmd_16bit = yaml_command_sixteenbit_max;
	}
	// Return the result
	return cmd_16bit;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Check if the request if for the default setpoint
		if (newSetpoint.buttonID == REQUEST_DEFAULT_SETPOINT_BUTTON_ID)
		{
			setNewSetpoint(
					yaml_default_setpoint[0],
					yaml_default_setpoint[1],
					yaml_default_setpoint[2],
					yaml_default_setpoint[3]
				);
		}
		else
		{
			// Call the function for actually setting the setpoint
			setNewSetpoint(
					newSetpoint.x,
					newSetpoint.y,
					newSetpoint.z,
					newSetpoint.yaw
				);
		}
	}
}

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw)
{
	// Put the new setpoint into the class variable
	m_setpoint[0] = x;
	m_setpoint[1] = y;
	m_setpoint[2] = z;
	m_setpoint[3] = yaw;

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = x;
	msg.y   = y;
	msg.z   = z;
	msg.yaw = yaw;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.x   = m_setpoint[0];
	response.setpointWithHeader.y   = m_setpoint[1];
	response.setpointWithHeader.z   = m_setpoint[2];
	response.setpointWithHeader.yaw = m_setpoint[3];
	// Return
	return true;
}





// REQUEST INTEGRATOR STATE CHANGE CALLBACK
void requestIntegratorStateChangeCallback(const IntWithHeader& msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the request change
		int requested_change = msg.data;

		// Switch between the possible cases
		switch (requested_change)
		{
			case INTEGRATOR_FLAG_ON:
				setNewIntegratorState(INTEGRATOR_FLAG_ON);
				break;

			case INTEGRATOR_FLAG_OFF:
				setNewIntegratorState(INTEGRATOR_FLAG_OFF);
				break;

			case INTEGRATOR_FLAG_UNKNOWN:
				ROS_INFO("[CSONE CONTROLLER] Does not respond for the integrator to change to state INTEGRATOR_FLAG_UNKNOWN");
				break;

			case INTEGRATOR_FLAG_TOGGLE:
			{
				if (m_integratorState==INTEGRATOR_FLAG_OFF)
					setNewIntegratorState(INTEGRATOR_FLAG_ON);
				else
					setNewIntegratorState(INTEGRATOR_FLAG_OFF);
				break;
			}
			
			case INTEGRATOR_FLAG_RESET:
				// Reset the integrator value to zero
				m_newtons_intergal = 0.0;
				m_tau_x_integral   = 0.0;
				m_tau_y_integral   = 0.0;
				m_tau_z_integral   = 0.0;
				// Inform the user
				ROS_INFO("[CSONE CONTROLLER] integrator values were reset to zero.");
				break;

			default:
				ROS_INFO_STREAM("[CSONE CONTROLLER] The requested integrator state change of " << requested_change << " is not recognised.");
				break;
		}
	}
}

// CHANGE INTEGRATOR STATE FUNCTION
void setNewIntegratorState(int newIntegratorState){
	// Directly set this to the class variable
	m_integratorState = newIntegratorState;

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "IntWithHeader"
	IntWithHeader msg;
	// Fill in the integrator state
	msg.data = m_integratorState;
	// Publish the message
	m_integratorStateChangedPublisher.publish(msg);
}




// GET CURRENT INTEGRATOR STATE SERVICE CALLBACK
bool getCurrentIntegratorStateCallback(IntIntService::Request &request, IntIntService::Response &response)
{
	// Directly put the current integrator state into the response
	response.data = m_integratorState;
	// Return
	return true;
}


// REQUEST TIME DELAY CALLBACK
void requestTimeDelayChangeCallback(const IntWithHeader& msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the request change
		int requested_time_delay = msg.data;

		// Call the function for setting the time delay
		setNewTimeDelay(requested_time_delay);
	}
}

// CHANGE TIME DELAY FUNCTION
void setNewTimeDelay(int newTimeDelay)
{

	// ASSUMPTION: that the time delay is in MILLISECONDS

	// Compute the number of milliseconds per time step
	float delta_T_in_milliseconds = 1000.0 / yaml_control_frequency;

	// Convert the time delay to a number of time steps
	int time_delay_in_steps = int( (float(newTimeDelay) + 0.1) / delta_T_in_milliseconds );

	// Wrap this value into the allowed limits
	if (time_delay_in_steps<0)
		time_delay_in_steps=0;
	if(time_delay_in_steps>(m_x_inertial_buffer.size()-1))
		time_delay_in_steps=m_x_inertial_buffer.size()-1;

	// Put the time delay into the class variable
	m_time_delay_in_steps = time_delay_in_steps;

	// Inform the user about the change
	ROS_INFO_STREAM("[CSONE CONTROLLER] Time delay changed to " << time_delay_in_steps << " steps, i.e., " << float(time_delay_in_steps)*1000.0/yaml_control_frequency << " milliseconds" );
}

// REQUEST PITCH ERROR CALLBACK
// Using x from SetpointWithHeader for pitch error value
void requestPitchErrorChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the request change
		m_pitch_error_in_rad = newSetpoint.x * PI / 180.0;
	}
}

// REQUEST CONTROLLER PARAMETERS CHANGE CALLBACK
// Using x y z from SetpointWithHeader for k T alpha controller parameters
void requestControllerParametersChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		convertIntoDiscreteTimeParameters(newSetpoint.x, newSetpoint.y, newSetpoint.z);
		m_controller_state = 0.0;
	}
}

// REQUEST LAG CONTROLLER PARAMETERS CHANGE CALLBACK
// Using y z from SetpointWithHeader for T alpha controller parameters
void requestLagControllerParametersChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		convertLagIntoDiscreteTimeParameters(newSetpoint.y, newSetpoint.z);
		m_lag_controller_state = 0.0;
	}
}

// CHANGE CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void convertIntoDiscreteTimeParameters(float k, float T, float alpha)
{
	if (alpha > 0.0 && T > 0.0){
		// valid parameters
		m_A = pow(2.71828, (-1.0 / (yaml_control_frequency*alpha*T)));
		m_B = -alpha * T * (m_A - 1.0);
		m_C = k / (alpha * T) * (1.0 - 1.0 / alpha);
		m_D = k / alpha;
	}
	else{
		// if invalid parameters, act as a proportional controller with gain k
		m_A = 0.0;
		m_B = 0.0;
        m_C = 0.0;
        m_D = k;
	}


	ROS_INFO_STREAM("[CSONE CONTROLLER] Parameters changed to k="<<k<<", T="<<T<<", alpha="<<alpha);
	ROS_INFO_STREAM("[CSONE CONTROLLER] Matrices changed to A="<<m_A<<", B="<<m_B<<", C="<<m_C<<", D="<<m_D);


}

// CHANGE LAG CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void convertLagIntoDiscreteTimeParameters(float T, float alpha)
{
	if (alpha > 0.0 && T > 0.0){
		// valid parameters
        m_lag_A = pow(2.71828, (-1.0/(yaml_control_frequency*alpha*T)));
        m_lag_B = -alpha * T * (m_lag_A - 1.0);
        m_lag_C = (alpha - 1.0) / (alpha * T);
    	m_lag_D = 1.0;
    }
    else{
		// if invalid parameters, act as a proportional controller with gain alpha
        m_lag_A = 0.0;
        m_lag_B = 0.0;
        m_lag_C = 0.0;
    	m_lag_D = alpha;
    }

	ROS_INFO_STREAM("[CSONE CONTROLLER] Lag parameters changed to T="<<T<<", alpha="<<alpha);
	ROS_INFO_STREAM("[CSONE CONTROLLER] Lag matrices changed to A="<<m_lag_A<<", B="<<m_lag_B<<", C="<<m_lag_C<<", D="<<m_lag_D);


}


//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , commandReceived.shouldCheckForAgentID , commandReceived.agentIDs );

	if (isRevelant)
	{
		// Extract the data from the message
		int custom_button_index = commandReceived.button_index;
		float float_data        = commandReceived.float_data;
		//std::string string_data = commandReceived.string_data;

		// Switch between the button pressed
		switch(custom_button_index)
		{

			// > FOR CUSTOM BUTTON 1
			case 1:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[CSONE CONTROLLER] Button 1 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 1
				
				break;
			}

			// > FOR CUSTOM BUTTON 2
			case 2:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[CSONE CONTROLLER] Button 2 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 2

				break;
			}

			// > FOR CUSTOM BUTTON 3
			case 3:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[CSONE CONTROLLER] Button 3 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 3

				break;
			}

			default:
			{
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[CSONE CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.float_data = " << float_data );
				break;
			}
		}
	}
}





//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------


// TIMER CALLBACK FOR SENDING THE LOAD YAML REQUEST
void timerCallback_initial_load_yaml(const ros::TimerEvent&)
{
	// Create a node handle to the selected parameter service
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	// Create the service client as a local variable
	ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// Create the service call as a local variable
	LoadYamlFromFilename loadYamlFromFilenameCall;
	// Specify the Yaml filename as a string
	loadYamlFromFilenameCall.request.stringWithHeader.data = "CsoneController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyDefaultControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
	// Inform the user
		ROS_ERROR("[CSONE CONTROLLER] The request load yaml file service call failed.");
	}
}


// CALLBACK NOTIFYING THAT THE YAML PARAMETERS ARE READY TO BE LOADED
void isReadyCsoneControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Initialise a local variable for the namespace
		std::string namespace_to_use;
		// Load from the respective parameter service
		switch(parameter_service_to_load_from)
		{
			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				ROS_INFO("[CSONE CONTROLLER] Now fetching the CsoneController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[CSONE CONTROLLER] Now fetching the CsoneController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[CSONE CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchCsoneControllerYamlParameters(nodeHandle_to_use);
	}
}


// LOADING OF THE YAML PARAMTERS
void fetchCsoneControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// CsoneController.yaml

	// Add the "CsoneController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "CsoneController");



	// GET THE PARAMETERS:

	// > The mass of the crazyflie
	yaml_cf_mass_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// > The frequency at which the "computeControlOutput" is being called,
	//   as determined by the frequency at which the motion capture system
	//   provides position and attitude data
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor
	//   command to thrust force in Newtons
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// > The min and max for saturating 16 bit thrust commands
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");

	// The default setpoint, the ordering is (x,y,z,yaw),
	// with unit [meters,meters,meters,radians]
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "default_setpoint", yaml_default_setpoint, 4);

	// Boolean indiciating whether the "Debug Message" of this agent
	// should be published or not
	yaml_shouldPublishDebugMessage = getParameterBool(nodeHandle_for_paramaters, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should
	// be displayed or not
	yaml_shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_paramaters, "shouldDisplayDebugInfo");

	// The LQR Controller parameters
	// The LQR Controller parameters for "LQR_MODE_RATE"
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "gainMatrixThrust_NineStateVector", yaml_gainMatrixThrust_NineStateVector, 9);
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "gainMatrixRollRate",               yaml_gainMatrixRollRate,               9);
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "gainMatrixPitchRate",              yaml_gainMatrixPitchRate,              9);
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "gainMatrixYawRate",                yaml_gainMatrixYawRate,                9);

	// Integrator gains
	yaml_integratorGain_forThrust = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forThrust");
	yaml_integratorGain_forTauXY  = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forTauXY");
	yaml_integratorGain_forTauYaw = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forTauYaw");


	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[CSONE CONTROLLER] DEBUGGING: the fetched CsoneController/mass = " << yaml_cf_mass_in_grams);



	// PROCESS THE PARAMTERS

	// > Compute the feed-forward force that we need to counteract
	//   gravity (i.e., mg) in units of [Newtons]
	m_cf_weight_in_newtons = yaml_cf_mass_in_grams * 9.81/1000.0;

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[CSONE CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
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
	ros::init(argc, argv, "CsoneControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "CsoneControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[CSONE CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



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
		ROS_ERROR("[CSONE CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[CSONE CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// NOTES:
	// > The parameters that are specified thorugh the *.yaml files
	//   are managed by a separate node called the "Parameter Service"
	// > A separate node is used for reasons of speed and generality
	// > To allow for a distirbuted architecture, there are two
	//   "ParamterService" nodes that are relevant:
	//   1) the one that is nested under the "m_agentID" namespace
	//   2) the one that is nested under the "m_coordID" namespace
	// > The following lines of code create the namespace (as strings)
	//   to there two relevant "ParameterService" nodes.
	// > The node handles are also created because they are needed
	//   for the ROS Subscriptions that follow.

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[CSONE CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[CSONE CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "CsoneController", 1, isReadyCsoneControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("CsoneController", 1, isReadyCsoneControllerYamlCallback);



	// GIVE YAML VARIABLES AN INITIAL VALUE
	// This can be done either here or as part of declaring the
	// variables in the header file




	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// The yaml files for the controllers are not added to
	// "Parameter Service" as part of launching.
	// The process for loading the yaml parameters is to send a
	// service call containing the filename of the *.yaml file,
	// and then a message will be received on the above subscribers
	// when the paramters are ready.
	// > NOTE IMPORTANTLY that by using a service client
	//   we stall the availability of this node until the
	//   paramter service is ready
	// > NOTE FURTHER that calling on the service directly from here
	//   often means the yaml file is not actually loaded. So we
	//   instead use a timer to delay the loading

	// Create a single-shot timer
	ros::Timer timer_initial_load_yaml = nodeHandle.createTimer(ros::Duration(2.0), timerCallback_initial_load_yaml, true);
	timer_initial_load_yaml.start();




    // PUBLISHERS AND SUBSCRIBERS

    // Instantiate the class variable "m_debugPublisher" to be a
    // "ros::Publisher". This variable advertises under the name
    // "DebugTopic" and is a message with the structure defined
    //  in the file "DebugMsg.msg" (located in the "msg" folder).
    m_debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestSetpointChangeSubscriber = nodeHandle.subscribe("RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	std::string namespace_to_coordinator;
	constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("CsoneControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestControllerParametersChangeSubscriber = nodeHandle.subscribe("RequestControllerParametersChange", 1, requestControllerParametersChangeCallback);
	ros::Subscriber requestLagControllerParametersChangeSubscriber = nodeHandle.subscribe("RequestLagControllerParametersChange", 1, requestLagControllerParametersChangeCallback);

	// Instantiate the class variable "m_setpointChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "SetpointChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current setpoint for this controller.
	m_setpointChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("SetpointChanged", 1);

	// Instantiate the local variable "getCurrentSetpointService" to be
	// a "ros::ServiceServer" type variable that advertises the service
	// called "GetCurrentSetpoint". This service has the input-output
	// behaviour defined in the "GetSetpointService.srv" file (located
	// in the "srv" folder). This service, when called, is provided with
	// an integer (that is essentially ignored), and is expected to respond
	// with the current setpoint of the controller. When a request is made
	// of this service the "getCurrentSetpointCallback" function is called.
	ros::ServiceServer getCurrentSetpointService = nodeHandle.advertiseService("GetCurrentSetpoint", getCurrentSetpointCallback);


	// Instantiate the local variable:
	// variable name:    "requestIntegatorStateChangeSubscriber"
	// variable type:    ros::Subscriber
	// name of message:  "RequestIntegratorStateChange"
	// The message has the structure defined in the file:
	//    "IntWithHeader.msg" (located in the "msg" folder).
	// The messages received by this subscriber typically come from
	// the "flying agent GUI".
	ros::Subscriber requestIntegatorStateChangeSubscriber = nodeHandle.subscribe("RequestIntegratorStateChange", 1, requestIntegratorStateChangeCallback);

	// Instantiate the class variable:
	// variable name:    "m_integratorStateChangedPublisher"
	// variable type:    ros::Publisher
	// name of message:  "IntegratorStateChanged"
	// The message has the structure defined in the file:
	//    "IntWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current state of the integator.
	m_integratorStateChangedPublisher = nodeHandle.advertise<IntWithHeader>("IntegratorStateChanged", 1);

	// Instantiate the local variable:
	// variable name:    "getCurrentIntegratorStateService"
	// variable type:    ros::ServiceServer
	// name of service:  "GetCurrentIntegratorState"
	// callback funcion: "getCurrentIntegratorStateCallback"
	// This service has the input-output behaviour defined in the file:
	//  "IntIntService.srv" (located in the "srv" folder)
	// This service, when called, is provided with an integer (that is
	// essentially ignored), and is expected to respond with the current
	// state of the integrator.
	ros::ServiceServer getCurrentIntegratorStateService = nodeHandle.advertiseService("GetCurrentIntegratorState", getCurrentIntegratorStateCallback);


	// Instantiate the local variable:
	// variable name:    "requestTimeDelayChangeSubscriber"
	// variable type:    ros::Subscriber
	// name of message:  "RequestTimeDelayChange"
	// The message has the structure defined in the file:
	//    "IntWithHeader.msg" (located in the "msg" folder).
	// The messages received by this subscriber typically come from
	// the "flying agent GUI".
	ros::Subscriber requestTimeDelayChangeSubscriber = nodeHandle.subscribe("RequestTimeDelayChange", 1, requestTimeDelayChangeCallback);


	// Instantiate the local variable:
	// variable name:    "requestPitchErrorChangeSubscriber"
	// variable type:    ros::Subscriber
	// name of message:  "RequestPitchErrorChange"
	// The message has the structure defined in the file:
	//    "SetpointWithHeader.msg" (located in the "msg" folder).
	// The messages received by this subscriber typically come from
	// the "flying agent GUI".
	ros::Subscriber requestPitchErrorChangeSubscriber = nodeHandle.subscribe("RequestPitchErrorChange", 1, requestPitchErrorChangeCallback);


    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "CsoneController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("CsoneController", calculateControlOutput);

    // Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "GUIButton" topic and calls the class
    // function "customCommandReceivedCallback" each time a messaged is received on this topic
    // and the message received is passed as an input argument to the callback function.
    ros::Subscriber customCommandReceivedSubscriber = nodeHandle.subscribe("CustomButtonPressed", 1, customCommandReceivedCallback);

    // Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	//std::string namespace_to_coordinator;
	//constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	//ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber customCommandReceivedSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("CsoneControllerService/CustomButtonPressed", 1, customCommandReceivedCallback);



    // Print out some information to the user.
    ROS_INFO("[CSONE CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
