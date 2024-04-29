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
//    A class for simulating a single Quadrotor
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "classes/QuadrotorSimulator.h"





// EMPTY CONSTRUCTOR
QuadrotorSimulator::QuadrotorSimulator () :
	m_gen_thrust( (std::random_device())() ),
	m_dist_thrust(-0.03,0.03),
	m_gen_body_x( (std::random_device())() ),
	m_dist_body_x(-0.1,0.1),
	m_gen_body_y( (std::random_device())() ),
	m_dist_body_y(-0.1,0.1),
	m_gen_body_z( (std::random_device())() ),
	m_dist_body_z(-0.1,0.1)
{
	// Set the default ID string
	this->set_id_string( "CF00" );
	// Set the default mass
	this->set_mass_in_kg( 0.032 );
}

// DEFAULT CONSTRUCTOR
QuadrotorSimulator::QuadrotorSimulator ( std::string id ) :
	m_gen_thrust( (std::random_device())() ),
	m_dist_thrust(-0.03,0.03),
	m_gen_body_x( (std::random_device())() ),
	m_dist_body_x(-0.1,0.1),
	m_gen_body_y( (std::random_device())() ),
	m_dist_body_y(-0.1,0.1),
	m_gen_body_z( (std::random_device())() ),
	m_dist_body_z(-0.1,0.1)
{
	// Set the input argument to the ID string
	this->set_id_string( id );
	//this->m_id_string = id;
	// Set the default mass
	this->set_mass_in_kg( 0.032 );
	//this->m_mass_in_kg = mass;
}

// PARAMETERIZED CONSTRUCTOR
QuadrotorSimulator::QuadrotorSimulator ( std::string id , float mass ) :
	m_gen_thrust( (std::random_device())() ),
	m_dist_thrust(-0.1*mass*GRAVITY,0.1*mass*GRAVITY),
	m_gen_body_x( (std::random_device())() ),
	m_dist_body_x(-0.1,0.1),
	m_gen_body_y( (std::random_device())() ),
	m_dist_body_y(-0.1,0.1),
	m_gen_body_z( (std::random_device())() ),
	m_dist_body_z(-0.1,0.1)
{
	// Set the input argument to the ID string
	this->set_id_string( id );
	// Set the input argument to the mass
	this->set_mass_in_kg( mass );
}




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


// Function to get the ID as a string
std::string QuadrotorSimulator::get_id_string()
{
	return this->m_id_string;
}

// Function to set the ID as a string
void QuadrotorSimulator::set_id_string(std::string new_id_string)
{
	this->m_id_string = new_id_string;
}

// Function to get the mass in kilograms
float QuadrotorSimulator::get_mass_in_kg()
{
	return this->m_mass_in_kg;
}

// Function to set the mass in kilograms
void QuadrotorSimulator::set_mass_in_kg(float new_mass_in_kg)
{
	this->m_mass_in_kg = new_mass_in_kg;
}

// Function to get the flying state
bool QuadrotorSimulator::get_isFlying()
{
	return this->m_isFlying;
}

// Function to simulate the quadrotor for one time step
void QuadrotorSimulator::simulate_for_one_time_step(float deltaT)
{
	// Only simulate if flying
	if (this->m_isFlying)
	{
		// --------------------------------------- //
		// PRE-COMPUTATIONS

		// Compute the square of "deltaT"
		float deltaT2 = deltaT * deltaT;

		// Compute the reciprocal of the mass
		float mass_recip = 1.0 / this->m_mass_in_kg;


		// --------------------------------------- //
		// ADD RANDOM PERTURBATION TO THE COMMANDS
		float command_body_rate_x = m_dist_body_x(m_gen_body_x) + this->m_current_command_body_rate_x;
		float command_body_rate_y = m_dist_body_y(m_gen_body_y) + this->m_current_command_body_rate_y;
		float command_body_rate_z = m_dist_body_z(m_gen_body_z) + this->m_current_command_body_rate_z;

		float command_total_thrust = m_dist_thrust(m_gen_thrust) + this->m_current_command_total_thurst;
		


		// --------------------------------------- //
		// YAW
		// Copy the current yaw into a local variable
		float current_yaw = this->m_euler_angles[2];
		// Simulate forward yaw
		float new_yaw = current_yaw + command_body_rate_z * deltaT;
		// Compute the yaw to be used for linearising
		float yaw_for_linearisation = 0.5 * (current_yaw+new_yaw);
		float sin_yaw = sin(yaw_for_linearisation);
		float cos_yaw = cos(yaw_for_linearisation);



		// --------------------------------------- //
		// PITCH
		// Copy the current pitch into a local variable
		float current_pitch = this->m_euler_angles[1];
		// Simulate forward pitch
		float new_pitch = current_pitch + command_body_rate_y * deltaT;
		// Compute the pitch to be used for linearising
		float pitch_for_linearisation = 0.5 * (current_pitch+new_pitch);
		float sin_pitch = sin(pitch_for_linearisation);
		float cos_pitch = cos(pitch_for_linearisation);



		// --------------------------------------- //
		// ROLL
		// Copy the current roll into a local variable
		float current_roll = this->m_euler_angles[0];
		// Simulate forward roll
		float new_roll = current_roll + command_body_rate_x * deltaT;
		// Compute the roll to be used for linearising
		float roll_for_linearisation = 0.5 * (current_roll+new_roll);
		float sin_roll = sin(roll_for_linearisation);
		float cos_roll = cos(roll_for_linearisation);



		// --------------------------------------- //
		// VERTICAL HEIGHT

		// Compute vertical component of total thrust,
		// taking into account gravity
		float thrust_vertical = command_total_thrust*cos_roll*cos_pitch - this->m_mass_in_kg*GRAVITY;

		// Simulate forward the vertical height
		float new_z_dot = this->m_velocity[2] + mass_recip*deltaT*thrust_vertical;
		float new_z     = this->m_position[2] + deltaT*this->m_velocity[2] + 0.5*mass_recip*deltaT2*thrust_vertical;



		// --------------------------------------- //
		// HORIZONTAL PLANE

		// Rotate the total thrust into INTERTIAL x and y components
		float thrust_x = ( sin_yaw*sin_roll + cos_yaw*sin_pitch*cos_roll) * command_total_thrust;
		float thrust_y = (-cos_yaw*sin_roll + sin_yaw*sin_pitch*cos_roll) * command_total_thrust;

		// Simulate forward the INERTIAL x direction
		float new_x_dot = this->m_velocity[0] + mass_recip*deltaT*thrust_x;
		float new_x     = this->m_position[0] + deltaT*this->m_velocity[0] + 0.5*mass_recip*deltaT2*thrust_x;

		// Simulate forward the INERTIAL y direction
		float new_y_dot = this->m_velocity[1] + mass_recip*deltaT*thrust_y;
		float new_y     = this->m_position[1] + deltaT*this->m_velocity[1] + 0.5*mass_recip*deltaT2*thrust_y;
		


		// --------------------------------------- //
		// UPDATE STATE WITH THE NEW VALUES
		// > For the positions
		this->m_position[0] = new_x;
		this->m_position[1] = new_y;
		this->m_position[2] = new_z;
		// > For the linear velocities
		this->m_velocity[0] = new_x_dot;
		this->m_velocity[1] = new_y_dot;
		this->m_velocity[2] = new_z_dot;
		// > For the Euler angles
		this->m_euler_angles[0] = new_roll;
		this->m_euler_angles[1] = new_pitch;
		this->m_euler_angles[2] = new_yaw;
		// > For the Euler angular velocities
		this->m_euler_velocities[0] = command_body_rate_x;
		this->m_euler_velocities[1] = command_body_rate_y;
		this->m_euler_velocities[2] = command_body_rate_z;



		// --------------------------------------- //
		// CHECK FOR HIGH PITCH OR ROLL ANGLE
		if ( (new_roll>(80*DEG2RAD)) || (new_pitch>(80*DEG2RAD)) )
		{
			// Reset the state
			this->reset();
			// Inform the user
			ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" reset due to roll or pitch exceeding 80 degrees." );
		}

	} // END OF: "if (this->m_isFlying)"
}


// Function to reset the quadrotor
void QuadrotorSimulator::reset()
{
	// Update the flying state to be not flying
	this->m_isFlying = false;

	// Set the state back to the default initial state
	// > For the position
	this->m_position[0] = this->m_reset_position_x;
	this->m_position[1] = this->m_reset_position_y;
	this->m_position[2] = this->m_reset_position_z;
	// > For the linear velocity
	this->m_velocity[0] = 0.0;
	this->m_velocity[1] = 0.0;
	this->m_velocity[2] = 0.0;
	// > For the euler angles
	this->m_euler_angles[0] = 0.0;
	this->m_euler_angles[1] = 0.0;
	this->m_euler_angles[2] = this->m_reset_euler_angle_yaw;
	// > For the euler anglular velocities
	this->m_euler_velocities[0] = 0.0;
	this->m_euler_velocities[1] = 0.0;
	this->m_euler_velocities[2] = 0.0;
}

// Function to update the commanding agent id
void QuadrotorSimulator::update_commanding_agent_id( int new_commanding_agent_id )
{
	// Inform the user
	ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" received request to connect to commanding agent ID " << new_commanding_agent_id );

	if( new_commanding_agent_id <= 0 )
	{
		// Set the id to the non-connected value
		this->m_commanding_agent_id = -1;
		// Unsubscribe from the commands
		this->unsubscribe_from_commanding_agent();
		// Reset the state
		this->reset();
	}
	else
	{
		// Only need to do something if the ID changes
		if ( new_commanding_agent_id != this->m_commanding_agent_id )
		{
			// Set the agent id to the new value
			this->m_commanding_agent_id = new_commanding_agent_id;
			// Unsubscribe from the commands
			this->unsubscribe_from_commanding_agent();
			// Reset the state
			this->reset();
			// Subscribe to commands for the new ID
			this->subscribe_to_commanding_agent_id( new_commanding_agent_id );
		}
	}

}

// Function to set the reset state
void QuadrotorSimulator::setResetState_xyz_yaw(float x, float y, float z, float yaw)
{
	this->m_reset_position_x = x;
	this->m_reset_position_y = y;
	this->m_reset_position_z = z;
	this->m_reset_euler_angle_yaw = yaw;
}

// Function to set the parameters for the
// 16-bit command to thrust conversion
void QuadrotorSimulator::setParameters_for_16bitCommand_to_thrust_conversion(float polyCoeff0, float polyCoeff1, float polyCoeff2, float cmd_min, float cmd_max)
{
	// Set the conversion coefficients
	this->m_motorPoly[0] = polyCoeff0;
	this->m_motorPoly[0] = polyCoeff1;
	this->m_motorPoly[0] = polyCoeff2;
	// Set the min and max
	this->m_command_sixteenbit_min = cmd_min;
	this->m_command_sixteenbit_max = cmd_max;

}

// Function to print the details of this quadrotor
void QuadrotorSimulator::print_details()
{
	ROS_INFO_STREAM("Quadrotor with ID: \"" << this->m_id_string << "\", mass = " << this->m_mass_in_kg << " [kg], reset (x,y,z,yaw) = ( " << this->m_reset_position_x << ", " << this->m_reset_position_y << ", " << this->m_reset_position_z << ", " << this->m_reset_euler_angle_yaw << ")");
}







// PRIVATE FUNCTIONS


// Callback for receiving control commands
void QuadrotorSimulator::control_commands_callback( const dfall_pkg::ControlCommand & msg )
{
	// Get the type of the command
	int x_mode   = msg.xControllerMode;
	int y_mode   = msg.yControllerMode;
	int z_mode   = msg.zControllerMode;
	int yaw_mode = msg.yawControllerMode;

	int command_type = 0;
	if (
		   (x_mode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE)
		&& (y_mode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE)
		&& (z_mode==CF_ONBOARD_CONTROLLER_MODE_OFF)
		&& (yaw_mode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE)
	)
	{
		command_type = 2;
	}
	if (
		   (x_mode==CF_ONBOARD_CONTROLLER_MODE_OFF)
		&& (y_mode==CF_ONBOARD_CONTROLLER_MODE_OFF)
		&& (z_mode==CF_ONBOARD_CONTROLLER_MODE_OFF)
		&& (yaw_mode==CF_ONBOARD_CONTROLLER_MODE_OFF)
	)
	{
		command_type = 1;
	}


	// Switch based on the command type
	// Adapt to the new state
	switch(command_type)
	{
		case 1:
		{
			// Put the command into the class variables
			this->update_current_control_command_rate(msg);
			break;
		}
		
		case 2:
		{
			// Put the command into the class variables
			this->update_current_control_command_rate(msg);

			// If the quadrotor is currently flying...
			//if (this->m_isFlying)
			//{
			//	// Simulate the quadrotor by one time step
			//}
			break;
		}

		default:
		{
			if (this->m_isFlying)
			{
				// Set that the quadrotor is NOT flying
				this->m_isFlying = false;
				// Reset the state
				this->reset();
				// Inform the user
				ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" stopped simulating and reset becuase command is NOT of type MOTORS or ANGULAR_RATE");
			}
			break;
		}
	}
}


// Function to update the current control command
void QuadrotorSimulator::update_current_control_command_rate( const dfall_pkg::ControlCommand & msg )
{
	// Convert each motor command to [Newtons]
	float cmd1_in_newtons = this->convert_16_bit_motor_command_to_newtons( msg.motorCmd1 );
	float cmd2_in_newtons = this->convert_16_bit_motor_command_to_newtons( msg.motorCmd2 );
	float cmd3_in_newtons = this->convert_16_bit_motor_command_to_newtons( msg.motorCmd3 );
	float cmd4_in_newtons = this->convert_16_bit_motor_command_to_newtons( msg.motorCmd4 );

	// Hence update the total thrust
	this->m_current_command_total_thurst = cmd1_in_newtons + cmd2_in_newtons + cmd3_in_newtons + cmd4_in_newtons;

	// Update the body rate commands
	this->m_current_command_body_rate_x = msg.yControllerSetpoint;
	this->m_current_command_body_rate_y = msg.xControllerSetpoint;
	this->m_current_command_body_rate_z = msg.yawControllerSetpoint;
}

// Callback for receiving flying state updates
void QuadrotorSimulator::flying_state_update_of_commanding_agent_callback( const std_msgs::Int32& msg )
{
	// Inform the user
	//ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" received flying state of " << msg.data << " from the commanding agent" );

	// Update the class variable with the new state
	this->m_flying_state_of_commanding_agent = msg.data;

	// Adapt to the new state
	switch(this->m_flying_state_of_commanding_agent)
	{
		case STATE_MOTORS_OFF:
		case STATE_UNAVAILABLE:
		{
			// Set that the quadrotor is NOT flying
			this->m_isFlying = false;
			// Reset the state
			this->reset();
			// Inform the user
			ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" stopped simulating and reset");
			break;
		}

		case STATE_TAKE_OFF:
		case STATE_FLYING:
		//case STATE_LAND:
		{
			if (!(this->m_isFlying))
			{
				// Set that the quadrotor is flying
				this->m_isFlying = true;
				// Inform the user
				ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" started simulating");
			}
			break;
		}
	}
}


// Function to unsubscribe from the commanding agent
void QuadrotorSimulator::unsubscribe_from_commanding_agent()
{
	// Shutdown the subscribers
	controlCommandsSubscriber.shutdown();
	flyingStateUpdatesOfCommandingAgentSubscriber.shutdown();

	// Inform the user
	ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" is no longer unsubscribed to a commanding agent");
}

// Function to subscribe to the commanding agent
void QuadrotorSimulator::subscribe_to_commanding_agent_id( int commanding_agent_id )
{
	// Convert the agent ID to a zero padded string
	std::ostringstream str_stream;
	str_stream << std::setw(3) << std::setfill('0') << commanding_agent_id;
	std::string commanding_agent_id_as_string(str_stream.str());
	
	// Get a node handle
	// > NOTE that subsciptions below start with a "/",
	//   hence subscribing based on an absolute path
	ros::NodeHandle nodeHandle("~");

	// Subscribe to the control commands of the
	// commanding agent
	controlCommandsSubscriber = nodeHandle.subscribe("/dfall/agent" + commanding_agent_id_as_string + "/CrazyRadio/ControlCommand", 1, &QuadrotorSimulator::control_commands_callback, this );

	// Subscribe to the flying state updates of the
	// commanding agent
	flyingStateUpdatesOfCommandingAgentSubscriber = nodeHandle.subscribe("/dfall/agent" + commanding_agent_id_as_string + "/FlyingAgentClient/FlyingState", 1, &QuadrotorSimulator::flying_state_update_of_commanding_agent_callback, this );

	// Inform to user
	ROS_INFO_STREAM("[QUADROTOR SIMULATOR] Quadrotor \"" << this->m_id_string << "\" now subscribed to commands from agent with ID " << commanding_agent_id_as_string);
}


// Function to convert 16-bit motor command to Newtons
float QuadrotorSimulator::convert_16_bit_motor_command_to_newtons( int motor_command )
{
	// Convert the command to a float
	float cmd = fmod( float( motor_command ) , 65535.0 );

	// Saturate it
	if (cmd < this->m_command_sixteenbit_min)
		cmd = 0.0;
	else if (cmd > this->m_command_sixteenbit_max)
		cmd = m_command_sixteenbit_max;

	// Compute the thrust in [Newtons]
	float thrust = 0.0;
	if (cmd > 0.0)
		thrust = this->m_motorPoly[2] * cmd*cmd + this->m_motorPoly[1] * cmd + this->m_motorPoly[0];

	// Return the result
	return thrust;
}