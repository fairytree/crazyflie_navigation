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





// INCLUDE THE HEADER
#include "nodes/CrazyRadioEmulator.h"

void change_radio_state_to( int new_state )
{
	// Update the class variable
	m_radio_state = new_state;
	// Inform the user of the updated state
	switch (m_radio_state)
	{
		case CRAZY_RADIO_STATE_DISCONNECTED:
		{
			ROS_INFO("[CRAZY RADIO EMULATOR] radio state changed to DISCONNECTED)");
			break;
		}
		case CRAZY_RADIO_STATE_CONNECTING:
		{
			ROS_INFO("[CRAZY RADIO EMULATOR] radio state changed to CONNECTING)");
			break;
		}
		case CRAZY_RADIO_STATE_CONNECTED:
		{
			ROS_INFO("[CRAZY RADIO EMULATOR] radio state changed to CONNECTED)");
			break;
		}
	}
	// Publish the updated state
	publishCurrentRadioState();
}

void publishCurrentRadioState()
{
	std_msgs::Int32 msg;
	msg.data = m_radio_state;
	crazyRadioStatusPublisher.publish(msg);
}


// PERFORM CONNECT
void connect()
{
	// Declare a static timer instance
	static float connecting_duration = 1.0;
	static ros::NodeHandle nodeHandle("~");
	static ros::Timer timer_connecting = nodeHandle.createTimer(ros::Duration(connecting_duration), connected_callback, true);

	// > Stop any previous instance that might still be running
	timer_connecting.stop();

	// Update status to CONNECTING
	change_radio_state_to(CRAZY_RADIO_STATE_CONNECTING);
	// Inform the user
	ROS_INFO("[CRAZY RADIO EMULATOR] Connecting...");
	
	// > Set the period again (second argument is reset)
	timer_connecting.setPeriod( ros::Duration(connecting_duration), true);
	// > Start the timer again
	timer_connecting.start();
}

// PERFORM DISCONNECT
void disconnect()
{
	// Declare a static timer instance
	static float disconnecting_duration = 0.1;
	static ros::NodeHandle nodeHandle("~");
	static ros::Timer timer_disconnecting = nodeHandle.createTimer(ros::Duration(disconnecting_duration), disconnected_callback, true);

	// > Stop any previous instance that might still be running
	timer_disconnecting.stop();

	// Send a MOTOS-OFF command
	ROS_INFO("[CRAZY RADIO EMMULATOR] sending Motors OFF command before disconnecting");

	// Send the MOTORS-OFF command to the Flying Agent Client
	IntWithHeader msg;
	msg.shouldCheckForAgentID = false;
	msg.data = CMD_CRAZYFLY_MOTORS_OFF;
	flyingAgentClientCommandPublisher.publish(msg);

	// Inform the user that now disconnecting
	ROS_INFO("[CRAZY RADIO EMULATOR] Disconnecting from ...");

	// > Set the period again (second argument is reset)
	timer_disconnecting.setPeriod( ros::Duration(disconnecting_duration), true);
	// > Start the timer again
	timer_disconnecting.start();

	// Update status to DISCONNECTED
	change_radio_state_to(CRAZY_RADIO_STATE_DISCONNECTED);
}




// RESPOND TO CONNECTED CALLBACK
void connected_callback(const ros::TimerEvent&)
{
	// Update status to CONNECTED
	change_radio_state_to(CRAZY_RADIO_STATE_CONNECTED);

	// Send the MOTORS-OFF command to the Flying Agent Client
	IntWithHeader msg;
	msg.shouldCheckForAgentID = false;
	msg.data = CMD_CRAZYFLY_MOTORS_OFF;
	flyingAgentClientCommandPublisher.publish(msg);

	// Start the battery voltage updates
	m_timer_battery_voltage_updates.start();

	// Start the state estimate updates
	m_timer_state_estimate_update.start();
}

void connection_failed()
{
	// Inform the user
	ROS_INFO("[CRAZY RADIO EMMULATOR] Connection failed");
	// Update status to DISCONNECTED
	change_radio_state_to(CRAZY_RADIO_STATE_DISCONNECTED);
	// Stop the battery voltage updates
	m_timer_battery_voltage_updates.stop();
	// Stop the state estimate updates
	m_timer_state_estimate_update.stop();
}

void connection_lost()
{
	// Inform the user
	ROS_INFO("[CRAZY RADIO EMMULATOR] Connection lost");
	// Update status to DISCONNECTED
	change_radio_state_to(CRAZY_RADIO_STATE_DISCONNECTED);
	// Stop the battery voltage updates
	m_timer_battery_voltage_updates.stop();
	// Stop the state estimate updates
	m_timer_state_estimate_update.stop();
}


// RESPOND TO DISCONNECTED CALLBACK
void disconnected_callback(const ros::TimerEvent&)
{
	// Update status to DISCONNECTED
	change_radio_state_to(CRAZY_RADIO_STATE_DISCONNECTED);
	// Stop the battery voltage updates
	m_timer_battery_voltage_updates.stop();
	// Stop the state estimate updates
	m_timer_state_estimate_update.stop();
}



void crazyRadioCommandCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Respond to a "should connect" command
		if (msg.data == CMD_RECONNECT)
		{
			switch (m_radio_state)
			{
				case CRAZY_RADIO_STATE_DISCONNECTED:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to CONNECT (current status is DISCONNECTED)");
					connect();
					break;
				}

				case CRAZY_RADIO_STATE_CONNECTING:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to CONNECT (current status is CONNECTING)");
					//publishCurrentRadioState();
					break;
				}

				case CRAZY_RADIO_STATE_CONNECTED:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to CONNECT (current status is CONNECTED)");
					//publishCurrentRadioState();
					break;
				}
			}
			
		}
		// Respond to a "should disconnect" command
		else if (msg.data == CMD_DISCONNECT)
		{
			switch (m_radio_state)
			{
				case CRAZY_RADIO_STATE_CONNECTED:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to DISCONNECT (current status is CONNECTED)");
					disconnect();
					break;
				}

				case CRAZY_RADIO_STATE_CONNECTING:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to DISCONNECT (current status is CONNECTING)");
					//publishCurrentRadioState();
					break;
				}

				case CRAZY_RADIO_STATE_DISCONNECTED:
				{
					ROS_INFO("[CRAZY RADIO EMULATOR] received command to CONNECT (current status is DISCONNECTED)");
					//publishCurrentRadioState();
					break;
				}
			}
		}
	}
}



void controlCommandCallback(const ControlCommand & msg)
{
	// Directly re-publish the message
	controlCommandPublisher.publish(msg);
}


bool getCurrentCrazyRadioStatusServiceCallback(IntIntService::Request &request, IntIntService::Response &response)
{
	// Put the radio state into the response variable
    response.data = m_radio_state;
    // Return
    return true;
}


void timerCallback_update_battery_voltage(const ros::TimerEvent&)
{
	// Declare static paramters for the voltage
	static float voltage_current = 4.2;
	static float voltage_decrement = 0.001;
	static float voltage_max = 4.2;
	static float voltage_min = 3.4;

	// Update the voltage
	voltage_current = voltage_current - voltage_decrement;

	// Wrap the voltage from the min back up to the max
	if (voltage_current < voltage_min)
		voltage_current = voltage_max;

	// Publish the current voltage
	std_msgs::Float32 msg;
	msg.data = voltage_current;
	batteryVoltagePublisher.publish(msg);
}





void timerCallback_update_cfStateEstimate(const ros::TimerEvent&)
{
	// Declare static counter for publshing the state estimates
	static int counter_since_last_send = 3;

	// Increment the counter
	counter_since_last_send++;

	// Simulate the quadrotor for one time step
	m_quadrotor_sim.simulate_for_one_time_step( yaml_cfSimulation_deltaT_in_seconds );

	// Perform safety check if required
	if (yaml_isEnabled_strictSafety)
	{
		// Estimate the height at the next measurement
		float height_at_next_measurement = m_quadrotor_sim.m_position[2] + yaml_cfSimulation_deltaT_in_seconds * m_quadrotor_sim.m_velocity[2];
		// Turn-off if too high
		if (height_at_next_measurement > yaml_maxHeight_for_strictSafety_meters)
		{
			// Send the MOTORS-OFF command to the Flying Agent Client
			IntWithHeader msg;
			msg.shouldCheckForAgentID = false;
			msg.data = CMD_CRAZYFLY_MOTORS_OFF;
			flyingAgentClientCommandPublisher.publish(msg);
			// Inform the user
			ROS_ERROR_STREAM("[CRAZY RADIO] Height safety check failed, measured = " << height_at_next_measurement << ", max allowed = " << yaml_maxHeight_for_strictSafety_meters );
		}
	}

	// Local variable for the data of this quadrotor
	FlyingVehicleState quadrotor_data;

	// Fill in the details
	// > For the type
	quadrotor_data.type = FLYING_VEHICLE_STATE_TYPE_CRAZYFLIE_STATE_ESTIMATE;
	// > For the name
	quadrotor_data.vehicleName = m_quadrotor_sim.get_id_string();
	// > For the occulsion flag
	quadrotor_data.isValid = true;
	// > For position
	quadrotor_data.x = m_quadrotor_sim.m_position[0];
	quadrotor_data.y = m_quadrotor_sim.m_position[1];
	quadrotor_data.z = m_quadrotor_sim.m_position[2];
	// > For velocities
	quadrotor_data.vx = m_quadrotor_sim.m_velocity[0];
	quadrotor_data.vy = m_quadrotor_sim.m_velocity[1];
	quadrotor_data.vz = m_quadrotor_sim.m_velocity[2];
	// > For euler angles
	quadrotor_data.roll  = m_quadrotor_sim.m_euler_angles[0];
	quadrotor_data.pitch = m_quadrotor_sim.m_euler_angles[1];
	quadrotor_data.yaw   = m_quadrotor_sim.m_euler_angles[2];
	// > For euler angular rates
	quadrotor_data.rollRate  = m_quadrotor_sim.m_euler_velocities[0];
	quadrotor_data.pitchRate = m_quadrotor_sim.m_euler_velocities[1];
	quadrotor_data.yawRate   = m_quadrotor_sim.m_euler_velocities[2];
	// > For the acquiring time
	quadrotor_data.acquiringTime = 0.0;

	// Publish the current state on the "CFSimulationState"
	// topic that is used by the MocapEmulator node
	cfSimulationStatePublisher.publish(quadrotor_data);


	// Publish the current state on the "CFStateEstimate"
	// topic that emulates the Crazyflie
	// > If counter indicates to do so
	if (counter_since_last_send >= yaml_cfSimulation_stateEstimate_sendEvery)
	{
		cfStateEstimatorPublisher.publish(quadrotor_data);
		// Reset the counter
		counter_since_last_send = 0;
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



void isReadyBatteryMonitorYamlCallback(const IntWithHeader & msg)
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
				ROS_INFO("[CRAZY RADIO EMULATOR] Now fetching the BatteryMonitor YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[CRAZY RADIO EMULATOR] Now fetching the BatteryMonitor YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[CRAZY RADIO EMULATOR] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchBatteryMonitorYamlParameters(nodeHandle_to_use);
	}
}






void fetchBatteryMonitorYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// BatteryMonitor.yaml

	// Add the "BatteryMonitor" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "BatteryMonitor");



	// Frequency of requesting the battery voltage, in [milliseconds]
	yaml_battery_polling_period_in_seconds = getParameterFloat(nodeHandle_for_paramaters,"battery_polling_period");
	yaml_battery_polling_period_in_seconds = yaml_battery_polling_period_in_seconds * 0.001;

	// // Battery thresholds while in the "motors off" state, in [Volts]
	// yaml_battery_voltage_threshold_lower_while_standby = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_lower_while_standby");
	// yaml_battery_voltage_threshold_upper_while_standby = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_upper_while_standby");

	// // Battery thresholds while in the "flying" state, in [Volts]
	// yaml_battery_voltage_threshold_lower_while_flying = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_lower_while_flying");
	// yaml_battery_voltage_threshold_upper_while_flying = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_upper_while_flying");

	// // Delay before changing the state of the battery, in [number of measurements]
	// // > Note that the delay in seconds therefore depends on the polling period
	// yaml_battery_delay_threshold_to_change_state = getParameterInt(nodeHandle_for_paramaters,"battery_delay_threshold_to_change_state");



	// DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] DEBUGGING: the fetched BatteryMonitor/battery_polling_period = " << yaml_battery_polling_period_in_seconds);



	// PROCESS ANY OF THE FETCHED PARAMETERS AS NECESSARY
}





void fetchCrazyRadioConfigYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// BatteryMonitor.yaml

	// Add the "CrazyRadioConfig" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "CrazyRadioConfig");

	// FLAG FOR WHETHER TO USE HEIGHT AS A TRIGGER FOR
	// PUBLISHING A MOTORS-OFF COMMAND
	yaml_isEnabled_strictSafety = getParameterBool(nodeHandle_for_paramaters,"isEnabled_strictSafety");
	yaml_maxHeight_for_strictSafety_meters = getParameterFloat(nodeHandle_for_paramaters,"maxHeight_for_strictSafety_meters");

	// SIMULATION FREQUENCY FOR EMULATING A CRAZYFLIE [Hertz]
	float yaml_cfSimulation_frequency = getParameterFloat(nodeHandle_for_paramaters,"cfSimulation_frequency");

	// HOW OFTEN TO PUBLISH THE SIMULATION STATE AS AN ONBOARD ESTIMATE
	int yaml_cfSimulation_stateEstimate_sendEvery = getParameterInt(nodeHandle_for_paramaters,"cfSimulation_stateEstimate_sendEvery");

	// THE COEFFICIENT OF THE 16-BIT COMMAND TO THRUST CONVERSION
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// THE MIN AND MAX FOR SATURATING THE 16-BIT THRUST COMMANDS
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");



	// DEBUGGING: Print out one of the parameters that was loaded
	//ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] DEBUGGING: the fetched CrazyRadioConfig/cfSimulation_stateEstimate_sendEvery = " << yaml_cfSimulation_stateEstimate_sendEvery);



	// PROCESS THE PARAMTERS
	// Convert from Hertz to second
	yaml_cfSimulation_deltaT_in_seconds = 1.0 / yaml_cfSimulation_frequency;

	// DEBUGGING: Print out one of the processed values
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] DEBUGGING: after processing yaml_cfSimulation_deltaT_in_seconds = " << yaml_cfSimulation_deltaT_in_seconds);
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
	ros::init(argc, argv, "CrazyRadio");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "BatteryMonitor" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] ros::this_node::getNamespace() =  " << m_namespace);



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
		ROS_ERROR("[CRAZY RADIO EMULATOR] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
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
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber batteryMonitor_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "BatteryMonitor", 1, isReadyBatteryMonitorYamlCallback);
	ros::Subscriber batteryMonitor_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("BatteryMonitor", 1, isReadyBatteryMonitorYamlCallback);


	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// Call the class function that loads the parameters for this class.
	fetchBatteryMonitorYamlParameters(nodeHandle_to_own_agent_parameter_service);

	// Call the class function that loads the parameters for this class.
	fetchCrazyRadioConfigYamlParameters(nodeHandle);



	// INITIALISE TIMER FOR THE BATTERY VOLTAGE
	m_timer_battery_voltage_updates = nodeHandle.createTimer(ros::Duration(yaml_battery_polling_period_in_seconds), timerCallback_update_battery_voltage, false);
	// > And stop it immediately
	m_timer_battery_voltage_updates.stop();


	// INITIALISE TIMER FOR THE STATE ESTIMATE PUBLISHER TIMER
	m_timer_state_estimate_update = nodeHandle.createTimer(ros::Duration(yaml_cfSimulation_deltaT_in_seconds), timerCallback_update_cfStateEstimate, false);
	// > And stop it immediately
	m_timer_state_estimate_update.stop();




	// INITIALISE THE QUADROTOR SIMULATOR
	// > First convert the agentID to a zero-padded string
	// Convert the agent ID to a zero padded string
	std::ostringstream str_stream;
	str_stream << std::setw(2) << std::setfill('0') << m_agentID;
	std::string agentID_as_string(str_stream.str());
	// > Initialise the quadrotor simulator class variable
	m_quadrotor_sim = QuadrotorSimulator( "CF" + agentID_as_string , 0.032  );
	// Set the agent id of quadrotor simulator
	// > This determines which message topic the quadrotor
	//   simulator subscribes to for commands
	m_quadrotor_sim.update_commanding_agent_id( m_agentID );
	// Set the reset state
	m_quadrotor_sim.setResetState_xyz_yaw(0.0,0.0,0.0,0.0);
	// Set the parameters for the 16-bit command to thrust conversion
	m_quadrotor_sim.setParameters_for_16bitCommand_to_thrust_conversion(yaml_motorPoly[0], yaml_motorPoly[1], yaml_motorPoly[2], yaml_command_sixteenbit_min, yaml_command_sixteenbit_max);
	// Reset the quadrotor
	m_quadrotor_sim.reset();
	// > Inform the user
	ROS_INFO_STREAM("[CRAZY RADIO EMULATOR] Initilised quadrotor simulation with ID = " << m_quadrotor_sim.get_id_string() << ", and mass in kg = " << m_quadrotor_sim.get_mass_in_kg() );
	ROS_INFO("[CRAZY RADIO EMULATOR] Initilised quadrotor simulation with details:");
	m_quadrotor_sim.print_details();




	// PUBLISHERS

	// Get a node handle to the Flying Agent Client
	std::string namespace_to_FlyingAgentClient = m_namespace + "/FlyingAgentClient";
	ros::NodeHandle nodeHandle_to_FlyingAgentClient(namespace_to_FlyingAgentClient);

	// Publisher for the status of the radio connection
	crazyRadioStatusPublisher = nodeHandle.advertise<std_msgs::Int32>("CrazyRadioStatus",1);

	// Publisher for sending a "Flying Agent Client Command"
	flyingAgentClientCommandPublisher = nodeHandle_to_FlyingAgentClient.advertise<IntWithHeader>("Command",1);

	// Publisher for the filtered battery voltage
	batteryVoltagePublisher = nodeHandle.advertise<std_msgs::Float32>("CFBattery",1);

	// Publisher for the onboard state estimate
    cfStateEstimatorPublisher = nodeHandle.advertise<FlyingVehicleState>("CFStateEstimate",1);

	// Publisher for the control commands
	// > Note this is not needed for the real CrazyRadio node because
	//   this command goes out over the radio, but for emulation we publish
	//   the control command
	controlCommandPublisher = nodeHandle.advertise<ControlCommand>("ControlCommand",1);

	// Publisher for the simulation state
	// > This is used by the MocapEmulator node
	cfSimulationStatePublisher = nodeHandle.advertise<FlyingVehicleState>("CFSimulationState",1);


	// SUBSCRIBERS

	// Get a node handle to the coordintor
	std::string namespace_to_coordinator;
	constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);

	// Get a node handle to the Crazy Radio
	std::string namespace_to_crazyradio = m_namespace + "/CrazyRadio";
	ros::NodeHandle nodeHandle_to_crazyradio(namespace_to_crazyradio);

	// Subscriber to the commands for when to (dis-)connect the
	// CrazyRadio connection with the Crazyflie
	// > For the radio commands from the FlyingAgentClient of this agent
	ros::Subscriber crazyRadioCommandSubscriber = nodeHandle_to_FlyingAgentClient.subscribe("CrazyRadioCommand", 1, crazyRadioCommandCallback);
	// > For the radio command from the coordinator
	ros::Subscriber crazyRadioCommandFromCoordSubscriber = nodeHandle_to_coordinator.subscribe("FlyingAgentClient/CrazyRadioCommand", 1, crazyRadioCommandCallback);
	
	// Subscriber for the "Control Commands" received from the Flying Agent Client
	ros::Subscriber controlCommandsSubscriber = nodeHandle_to_FlyingAgentClient.subscribe("ControlCommand", 1, controlCommandCallback);



	// SERVICE SERVER FOR OTHERS TO GET THE CURRENT RADIO STATE
	// Advertise the service that return the "m_radio_state"
	// variable when called upon
	ros::ServiceServer getCurrentRadioStateService = nodeHandle.advertiseService("getCurrentCrazyRadioStatus", getCurrentCrazyRadioStatusServiceCallback);



	// Inform the user the this node is ready
	ROS_INFO("[CRAZY RADIO EMULATOR] Ready :-)");
	// Spin as a single-thread node
	ros::spin();

	return 0;
}