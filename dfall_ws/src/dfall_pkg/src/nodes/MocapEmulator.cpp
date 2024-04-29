//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
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





// INCLUDE THE HEADER
#include "nodes/MocapEmulator.h"





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

void timerCallback_mocap_publisher(const ros::TimerEvent&)
{
	// Initilise a "ViconData" struct
	// > This is defined in the "ViconData.msg" file
	ViconData mocapData;

	// Get the number of quadrotors
	unsigned int quadrotor_count = m_flying_fleet_states.size();

	// If there are any quadrotors:
	if (quadrotor_count>0)
	{

		// Iterate through the vector of quadrotors
		for(std::vector<FlyingVehicleState>::iterator it = m_flying_fleet_states.begin(); it != m_flying_fleet_states.end(); ++it)
		{
			// Access that current element as *it

			// Get the index if needed
			int idx = std::distance( m_flying_fleet_states.begin() , it );

			// Convert the state to global
			float local_x  = it->x;
			float local_y  = it->y;
			float local_z  = it->z;
			float global_x = local_x + (m_flying_fleet_areaBounds[idx].xmin + m_flying_fleet_areaBounds[idx].xmax) / 2.0;
			float global_y = local_y + (m_flying_fleet_areaBounds[idx].ymin + m_flying_fleet_areaBounds[idx].ymax) / 2.0;
			float global_z = local_z; // + (m_flying_fleet_areaBounds[idx].zmin + m_flying_fleet_areaBounds[idx].zmax) / 2.0;

			// Local variable for the data of this quadrotor
			FlyingVehicleState quadrotor_data;

			// Fill in the details
			// > For the type
			quadrotor_data.type = FLYING_VEHICLE_STATE_TYPE_MOCAP_MEASUREMENT;
			// > For the name
			quadrotor_data.vehicleName = it->vehicleName;
			// > For the occulsion flag
			quadrotor_data.isValid = it->isValid;
			// > For position
			quadrotor_data.x = global_x;
			quadrotor_data.y = global_y;
			quadrotor_data.z = global_z;
			// > For euler angles
			quadrotor_data.roll  = it->roll;
			quadrotor_data.pitch = it->pitch;
			quadrotor_data.yaw   = it->yaw;
			// > For the acquiring time
			quadrotor_data.acquiringTime = 0.0;

			// Push back into the Mocap Data variable
			mocapData.crazyflies.push_back(quadrotor_data);
		}
	}

	// Publish the Motion Capture data
	m_mocapDataPublisher.publish(mocapData);
}





// CALLBACK FOR RECEIVEING CRAZYFLIE SIMULATION DATA
// > This data is published by the CrazyRadioEmulator node
void cfSimulationStateCallback(const FlyingVehicleState & msg)
{
	// Extract the id string
	std::string id_as_string = msg.vehicleName;
	// Convert the id to an integer
	int id_as_int = std::stoi( id_as_string.substr( id_as_string.length()-2 ) );
	// Update the corresponding entry of the "m_flying_fleet_states"
	m_flying_fleet_states[id_as_int] = msg;
}




//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//     CCCC   OOO   N   N  TTTTT  EEEEE  X   X  TTTTT
//    C      O   O  NN  N    T    E       X X     T
//    C      O   O  N N N    T    EEE      X      T
//    C      O   O  N  NN    T    E       X X     T
//     CCCC   OOO   N   N    T    EEEEE  X   X    T
//    ----------------------------------------------------------------------------------


void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg)
{
	// Inform the user
    ROS_INFO("[MOCAP EMULATOR] Received message that the Context Database Changed");

    // Load the context for each quadrotor simulator
    loadContextForEachQuadrotor();
}



void loadContextForEachQuadrotor()
{

	// Iterate through the quadrotors
	// Iterate through the vector of quadrotors
	for(std::vector<FlyingVehicleState>::iterator it = m_flying_fleet_states.begin(); it != m_flying_fleet_states.end(); ++it)
	{
		// Access that current element as *it

		// Get the index if needed
		int idx = std::distance( m_flying_fleet_states.begin() , it );

		// Local variable for the service call
		CMQueryCrazyflieName contextCall;

		// Set the name of this quadrotor
		contextCall.request.crazyflieName = it->vehicleName;

		// Wait for the service to exist
		m_centralManagerService.waitForExistence(ros::Duration(0));

		// Local variable for the agent ID
		int new_agent_id = -1;

		// Call the service
		if (m_centralManagerService.call(contextCall))
		{
			// Get the context from the response
			CrazyflieContext new_context = contextCall.response.crazyflieContext;

			// Get the student ID from the response
			new_agent_id = contextCall.response.studentID;

			// Update the reset position of the quadrotor simulator
			// to be the center of the context
			// > Get the area into a local variable
			m_flying_fleet_areaBounds[idx] = new_context.localArea;
			// > Compute the X-Y coordinates
			//float new_reset_x = (area.xmin + area.xmax) / 2.0;
			//float new_reset_y = (area.ymin + area.ymax) / 2.0;
			// > Update the reset state
			//it->setResetState_xyz_yaw(new_reset_x,new_reset_y,0.0,0.0);

			// Print out the new context
			//ROS_INFO_STREAM("[MOCAP EMULATOR] Quadrotor simulator with ID \"" << it->get_id_string() << "\" connected to agent ID " << new_agent_id << " in database.");
		}
		else
		{
			// Let the user know that the "id_string" was not found
			// for this quadrotor simulation
			//ROS_INFO_STREAM("[MOCAP EMULATOR] Quadrotor simulator with ID \"" << it->get_id_string() << "\" does not appear in the database.");

			// Set the "new agent id" to reflect this
			new_agent_id = -1;
		}

		// Update the agent id of quadrotor simulator
		//it->update_commanding_agent_id( new_agent_id );
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

void fetchMocapEmulatorConfigYamlParameters()
{
	// Create a "ros::NodeHandle" type local variable
	// "nodeHandle_for_paramaters"
	// as the current node, the "~" indcates that "self"
	// is the node handle assigned to this variable.
	ros::NodeHandle nodeHandle_for_paramaters("~");

	// FREQUENCY OF THE MOTION CAPTURE EMULATOR [Hertz]
	yaml_mocap_frequency = getParameterFloat(nodeHandle_for_paramaters,"mocap_frequency");

	// THE COEFFICIENT OF THE 16-BIT COMMAND TO THRUST CONVERSION
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// THE MIN AND MAX FOR SATURATING THE 16-BIT THRUST COMMANDS
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");



	// DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[MOCAP EMULATOR] DEBUGGING: the fetched mocap_frequency = " << yaml_mocap_frequency);



	// PROCESS THE PARAMTERS
	// Convert from Hertz to second
	yaml_mocap_deltaT_in_seconds = 1.0 / yaml_mocap_frequency;

	// DEBUGGING: Print out one of the processed values
	ROS_INFO_STREAM("[MOCAP EMULATOR] DEBUGGING: after processing yaml_mocap_deltaT_in_seconds = " << yaml_mocap_deltaT_in_seconds);
}





//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
	// Starting the ROS-node
	ros::init(argc, argv, "ViconDataPublisher");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[MOCAP EMULATOR] ros::this_node::getNamespace() =  " << m_namespace);




	// LOAD THE YAML PARAMETERS

	// Call the class function that loads the parameters
	// from the "MocapEmulatorConfig.yaml" file.
	// > This is possible because that YAML file is added
	//   to the parameter service when launched via the
	//   "master.launch" file.
	fetchMocapEmulatorConfigYamlParameters();



	// CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
	ros::NodeHandle nodeHandle_dfall_root("/dfall");



	// ADD QUADROTORS TO THE FLEET

	for ( int i_quad=1 ; i_quad<4 ; i_quad++ )
	{
		// Compute the x-coordinate of the reset state
		float this_reset_x = float(i_quad-1) * 1.0 + 0.5;

		// Create a string for the ID, zero padded
		// > of length 2
		std::ostringstream str_stream_02;
		str_stream_02 << std::setw(2) << std::setfill('0') << i_quad;
		std::string this_quad_id_as_string(str_stream_02.str());
		// > of length 3
		std::ostringstream str_stream_03;
		str_stream_03 << std::setw(3) << std::setfill('0') << i_quad;
		std::string this_agent_id_as_string(str_stream_03.str());

		// Prepare a reset state of this flying vehicle
		FlyingVehicleState this_state;
		this_state.x       = 0.0f;
		this_state.y       = 0.0f;
		this_state.z       = 0.0f;
		this_state.roll    = 0.0f;
		this_state.pitch   = 0.0f;
		this_state.yaw     = 0.0f;
		this_state.isValid = false;
		this_state.vehicleName = "CF"+this_quad_id_as_string;

		// Push back into the vector of states
		m_flying_fleet_states.push_back(this_state);

		// Subscribe to the messages for this vehicle
		m_flying_fleet_state_subscribers.push_back( nodeHandle_dfall_root.subscribe("agent"+this_agent_id_as_string+"/CrazyRadio/CFSimulationState", 50, cfSimulationStateCallback) );

		// Set the Area Bounds
		AreaBounds this_area;
		this_area.xmin = -1.0f + this_reset_x;
		this_area.xmax =  1.0f + this_reset_x;
		this_area.ymin = -1.0f;
		this_area.ymax =  1.0f;
		this_area.zmin =  0.0f;
		this_area.zmax =  2.0f;
		m_flying_fleet_areaBounds.push_back( this_area );

		ROS_INFO_STREAM("this_reset_x = " << this_reset_x);

	}



	// INITIALISE THE MOTION CAPTURE PUBLISHER TIMER
	m_timer_mocap_publisher = nodeHandle.createTimer(ros::Duration(yaml_mocap_deltaT_in_seconds), timerCallback_mocap_publisher, false);
	// > And stop it immediately
	//m_timer_mocap_timeout_check.stop();



	// PUBLISHER FOR THE MOTION CAPTURE DATA
	// Instantiate the class variable "m_mocapDataPublisher" to be a
	// "ros::Publisher". This variable advertises under the name
	// "ViconData" and is a message with the structure defined
	//  in the file "ViconData.msg" (located in the "msg" folder).
    m_mocapDataPublisher = nodeHandle.advertise<ViconData>("ViconData", 1);



    // SUBSCRIBER FOR DATABASE CHANGES
    // > This is the database of tuples of the form:
    //   {student ID, flying zone, crazyflie name}
	m_centralManagerService = nodeHandle_dfall_root.serviceClient<CMQueryCrazyflieName>("CentralManagerService/QueryCrazyflieName", false);
	ros::Subscriber databaseChangedSubscriber = nodeHandle_dfall_root.subscribe("CentralManagerService/DBChanged", 1, crazyflieContextDatabaseChangedCallback);


	// Print out some information to the user.
    ROS_INFO("[MOCAP EMULATOR] Ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}