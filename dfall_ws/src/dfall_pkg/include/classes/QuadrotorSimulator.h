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
//    The service that manages the loading of YAML parameters
//
//    ----------------------------------------------------------------------------------




#ifndef QUADROTORSIMULATOR_H
#define QUADROTORSIMULATOR_H


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
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

// Include the DFALL message types
// #include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/ControlCommand.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace dfall_pkg;
using namespace std;




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------









class QuadrotorSimulator
{

//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

public:

	// Empty Constructor
	QuadrotorSimulator ();

	// Default Constructor
	QuadrotorSimulator ( std::string id );

	// Paramterised Constructor
	QuadrotorSimulator ( std::string id , float mass );



public:

	// The current state
	std::vector<float> m_position          = {0.0,0.0,0.0};
	std::vector<float> m_velocity          = {0.0,0.0,0.0};
	std::vector<float> m_euler_angles      = {0.0,0.0,0.0};
	std::vector<float> m_euler_velocities  = {0.0,0.0,0.0};


private:

	// Identifier String
	std::string m_id_string = "none";

	// Agent ID which command this quadrotor
	int m_commanding_agent_id = -1;

	// Mass of the quadrotor [kilograms]
	float m_mass_in_kg = 0.032;

	// The flying state
	bool m_isFlying = false;

	// The flying state of the commanding agent
	int m_flying_state_of_commanding_agent = STATE_MOTORS_OFF;

	// Subscribe for the radio commands of the commanding agent
	ros::Subscriber controlCommandsSubscriber;

	// Subscribe for the flying state updates of the
	// commanding agent
	ros::Subscriber flyingStateUpdatesOfCommandingAgentSubscriber;

	// Current control commands
	float m_current_command_total_thurst = 0.0;
	float m_current_command_body_rate_x = 0.0;
	float m_current_command_body_rate_y = 0.0;
	float m_current_command_body_rate_z = 0.0;

	// The reset state
	float m_reset_position_x = 0.0;
	float m_reset_position_y = 0.0;
	float m_reset_position_z = 0.0;
	float m_reset_euler_angle_yaw = 0.0;

	// THE COEFFICIENT OF THE 16-BIT COMMAND TO THRUST CONVERSION
	std::vector<float> m_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};

	// THE MIN AND MAX FOR SATURATING THE 16-BIT THRUST COMMANDS
	float m_command_sixteenbit_min = 1000.0;
	float m_command_sixteenbit_max = 65000.0;

	// RANDOM NUMBER GENERATORS
	std::mt19937 m_gen_thrust;
	std::uniform_real_distribution<float> m_dist_thrust;

	std::mt19937 m_gen_body_x;
	std::uniform_real_distribution<float> m_dist_body_x;

	std::mt19937 m_gen_body_y;
	std::uniform_real_distribution<float> m_dist_body_y;

	std::mt19937 m_gen_body_z;
	std::uniform_real_distribution<float> m_dist_body_z;



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


public:

	// Function to get the ID as a string
	std::string get_id_string();

	// Function to set the ID as a string
	void set_id_string(std::string new_id_string);

	// Function to get the mass in kilograms
	float get_mass_in_kg();

	// Function to set the mass in kilograms
	void set_mass_in_kg(float new_mass_in_kg);

	// Function to get the flying state
	bool get_isFlying();

	// Function to simulate the quadrotor for one time step
	void simulate_for_one_time_step(float deltaT);

	// Function to reset the quadrotor
	void reset();

	// Function to update the commanding agent id
	void update_commanding_agent_id( int new_commanding_agent_id );

	

	// Function to set the reset state
	void setResetState_xyz_yaw(float x, float y, float z, float yaw);

	// Function to set the parameters for the
	// 16-bit command to thrust conversion
	void setParameters_for_16bitCommand_to_thrust_conversion(float polyCoeff0, float polyCoeff1, float polyCoeff2, float cmd_min, float cmd_max);

	// Function to print the details of this quadrotor
	void print_details();


private:

	// Callback for receiving control commands
	void control_commands_callback( const dfall_pkg::ControlCommand & msg );

	// Function to update the current control command
	void update_current_control_command_rate( const dfall_pkg::ControlCommand & msg );

	// Callback for receiving flying state updates
	void flying_state_update_of_commanding_agent_callback( const std_msgs::Int32& msg );

	// Function to unsubscribe from the commanding agent
	void unsubscribe_from_commanding_agent();

	// Function to subscribe to the commanding agent
	void subscribe_to_commanding_agent_id( int commanding_agent_id );

	// Function to convert 16-bit motor command to Newtons
	float convert_16_bit_motor_command_to_newtons( int motor_command );

};

#endif // QUADROTORSIMULATOR_H