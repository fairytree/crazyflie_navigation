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





// INCLUDE THE HEADER
#include "nodes/AgentStatusForWebInterface.h"











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



// CALLBACKS FOR MESSAGES WITH THE STATUS OF THINGS

// > For the status of the crazyradio
void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
	m_crazyradio_status = msg.data;
}

// > For the battery level
void newBatteryLevelCallback(const std_msgs::Int32& msg)
{
	m_battery_level = msg.data;
}

// > For the flying state of the agent
void agentOperatingStateCallback(const std_msgs::Int32& msg)
{
	m_agent_operating_state = msg.data;
}

// > For the instant controller
void instantControllerChangedCallback(const std_msgs::Int32& msg)
{
	m_instant_controller = msg.data;
}

// > For the Default Controller Setpoint
void defaultControllerSetpointChangedCallback(const SetpointWithHeader& newSetpoint)
{
    m_setpoint_default[0] = newSetpoint.x;
    m_setpoint_default[1] = newSetpoint.y;
    m_setpoint_default[2] = newSetpoint.z;
    m_setpoint_default[3] = newSetpoint.yaw;
}

// > For the Student Controller Setpoint
void studentControllerSetpointChangedCallback(const SetpointWithHeader& newSetpoint)
{
    m_setpoint_student[0] = newSetpoint.x;
    m_setpoint_student[1] = newSetpoint.y;
    m_setpoint_student[2] = newSetpoint.z;
    m_setpoint_student[3] = newSetpoint.yaw;
}

// > For the Student Controller Debug Values
void studentControllerDebugValuesCallback(const DebugMsg& newDebugMsg)
{
    m_debug_values_student[0] = newDebugMsg.value_1;
    m_debug_values_student[1] = newDebugMsg.value_2;
    m_debug_values_student[2] = newDebugMsg.value_3;
    m_debug_values_student[3] = newDebugMsg.value_4;
    m_debug_values_student[4] = newDebugMsg.value_5;
    m_debug_values_student[5] = newDebugMsg.value_6;
    m_debug_values_student[6] = newDebugMsg.value_7;
    m_debug_values_student[7] = newDebugMsg.value_8;
    m_debug_values_student[8] = newDebugMsg.value_9;
    m_debug_values_student[9] = newDebugMsg.value_10;
}

// > For the State Estimate from the Crazyflie
void cfStateEstimateCallback(const FlyingVehicleState & newStateEstimate)
{
	// > For (x,y,z) positions
	m_cf_state_estimate_xyz_rpy[0] = newStateEstimate.x;
	m_cf_state_estimate_xyz_rpy[1] = newStateEstimate.y;
	m_cf_state_estimate_xyz_rpy[2] = newStateEstimate.z;
	// > For (roll,pitch,yaw) orientation
	m_cf_state_estimate_xyz_rpy[3] = newStateEstimate.roll;
	m_cf_state_estimate_xyz_rpy[4] = newStateEstimate.pitch;
	m_cf_state_estimate_xyz_rpy[5] = newStateEstimate.yaw;
}





// SERVICE CALLBACK FOR PROVIDING STATUS TO THE WEB INTERFACE
bool statusForWebInterfaceCallback(IntStringService::Request &request, IntStringService::Response &response)
{
	// Get the CrazyRadio status as a string
	std::string crazyradio_status_string;
	switch (m_crazyradio_status)
	{
		case CRAZY_RADIO_STATE_CONNECTED:
		{
			crazyradio_status_string = "connected";
			break;
		}
		case CRAZY_RADIO_STATE_CONNECTING:
		{
			crazyradio_status_string = "connecting";
			break;
		}
		case CRAZY_RADIO_STATE_DISCONNECTED:
		{
			crazyradio_status_string = "disconnected";
			break;
		}
		default:
		{
			crazyradio_status_string = "unknown";
			break;
		}
	}


	// Get the Battery level as a string
	std::string battery_level_string;
	switch (m_battery_level)
	{
		case BATTERY_LEVEL_000:
		{
			battery_level_string = "000";
			break;
		}
		case BATTERY_LEVEL_010:
		{
			battery_level_string = "010";
			break;
		}
		case BATTERY_LEVEL_020:
		{
			battery_level_string = "020";
			break;
		}
		case BATTERY_LEVEL_030:
		{
			battery_level_string = "030";
			break;
		}
		case BATTERY_LEVEL_040:
		{
			battery_level_string = "040";
			break;
		}
		case BATTERY_LEVEL_050:
		{
			battery_level_string = "050";
			break;
		}
		case BATTERY_LEVEL_060:
		{
			battery_level_string = "060";
			break;
		}
		case BATTERY_LEVEL_070:
		{
			battery_level_string = "070";
			break;
		}
		case BATTERY_LEVEL_080:
		{
			battery_level_string = "080";
			break;
		}
		case BATTERY_LEVEL_090:
		{
			battery_level_string = "090";
			break;
		}
		case BATTERY_LEVEL_100:
		{
			battery_level_string = "100";
			break;
		}
		case BATTERY_LEVEL_UNAVAILABLE:
		{
			battery_level_string = "unavailable";
			break;
		}
		default:
		{
			battery_level_string = "unavailable";
			break;
		}
	}

	// Get the Flying status as a string
	std::string flying_state_string;
	switch (m_agent_operating_state)
	{
		case STATE_MOTORS_OFF:
		{
			flying_state_string = "motorsoff";
			break;
		}
		case STATE_TAKE_OFF:
		{
			flying_state_string = "takeoff";
			break;
		}
		case STATE_FLYING:
		{
			flying_state_string = "flying";
			break;
		}
		case STATE_LAND:
		{
			flying_state_string = "land";
			break;
		}
		case STATE_UNAVAILABLE:
		{
			flying_state_string = "unavailable";
			break;
		}
		default:
		{
			flying_state_string = "unavailable";
			break;
		}
	}

	// Get the Flying status as a string
	std::string instant_controller_string;
	switch (m_instant_controller)
	{
		case DEFAULT_CONTROLLER:
		{
			instant_controller_string = "default";
			break;
		}
		case DEMO_CONTROLLER:
		{
			instant_controller_string = "demo";
			break;
		}
		case STUDENT_CONTROLLER:
		{
			instant_controller_string = "student";
			break;
		}
		case MPC_CONTROLLER:
		{
			instant_controller_string = "mpc";
			break;
		}
		case REMOTE_CONTROLLER:
		{
			instant_controller_string = "remote";
			break;
		}
		case TUNING_CONTROLLER:
		{
			instant_controller_string = "tuning";
			break;
		}
		case PICKER_CONTROLLER:
		{
			instant_controller_string = "picker";
			break;
		}
		case TEMPLATE_CONTROLLER:
		{
			instant_controller_string = "template";
			break;
		}
		case CSONE_CONTROLLER:
		{
			instant_controller_string = "csone";
			break;
		}
		case TESTMOTORS_CONTROLLER:
		{
			instant_controller_string = "testmotors";
			break;
		}
		default:
		{
			instant_controller_string = "none";
			break;
		}
	}

	// Concatenate the json together using a string stream
	std::stringstream ss;
	ss << "{";
	ss << "\u0022crazyradiostatus\u0022: \u0022" << crazyradio_status_string << "\u0022";
	ss << " , ";
	ss << "\u0022batterylevel\u0022: \u0022" << battery_level_string << "\u0022";
	ss << " , ";
	ss << "\u0022flyingstate\u0022: \u0022" << flying_state_string << "\u0022";
	ss << " , ";
	ss << "\u0022instantcontroller\u0022: \u0022" << instant_controller_string << "\u0022";
	ss << " , ";
	ss << "\u0022setpointdefault\u0022:";
	ss << "{";
	ss << "\u0022x\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_default[0];
	ss << " , ";
	ss << "\u0022y\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_default[1];
	ss << " , ";
	ss << "\u0022z\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_default[2];
	ss << " , ";
	ss << "\u0022yaw\u0022: " << std::setprecision(1) << std::fixed << m_setpoint_default[3]*RAD2DEG;
	ss << "}";
	ss << " , ";
	ss << "\u0022setpointstudent\u0022:";
	ss << "{";
	ss << "\u0022x\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_student[0];
	ss << " , ";
	ss << "\u0022y\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_student[1];
	ss << " , ";
	ss << "\u0022z\u0022: " << std::setprecision(3) << std::fixed << m_setpoint_student[2];
	ss << " , ";
	ss << "\u0022yaw\u0022: " << std::setprecision(1) << std::fixed << m_setpoint_student[3]*RAD2DEG;
	ss << "}";
	ss << " , ";
	ss << "\u0022stateestimate\u0022:";
	ss << "{";
	ss << "\u0022x\u0022: " << std::setprecision(3) << std::fixed << m_cf_state_estimate_xyz_rpy[0];
	ss << " , ";
	ss << "\u0022y\u0022: " << std::setprecision(3) << std::fixed << m_cf_state_estimate_xyz_rpy[1];
	ss << " , ";
	ss << "\u0022z\u0022: " << std::setprecision(3) << std::fixed << m_cf_state_estimate_xyz_rpy[2];
	ss << " , ";
	ss << "\u0022roll\u0022: "  << std::setprecision(1) << std::fixed << m_cf_state_estimate_xyz_rpy[3]*RAD2DEG;
	ss << " , ";
	ss << "\u0022pitch\u0022: " << std::setprecision(1) << std::fixed << m_cf_state_estimate_xyz_rpy[4]*RAD2DEG;
	ss << " , ";
	ss << "\u0022yaw\u0022: "   << std::setprecision(1) << std::fixed << m_cf_state_estimate_xyz_rpy[5]*RAD2DEG;
	ss << "}";
	ss << " , ";
	ss << "\u0022debugvaluesstudent\u0022:";
	ss << "{";
	ss << "\u0022value1\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[0];
	ss << " , ";
	ss << "\u0022value2\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[1];
	ss << " , ";
	ss << "\u0022value3\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[2];
	ss << " , ";
	ss << "\u0022value4\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[3];
	ss << " , ";
	ss << "\u0022value5\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[4];
	ss << " , ";
	ss << "\u0022value6\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[5];
	ss << " , ";
	ss << "\u0022value7\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[6];
	ss << " , ";
	ss << "\u0022value8\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[7];
	ss << " , ";
	ss << "\u0022value9\u0022: "  << std::setprecision(3) << std::fixed << m_debug_values_student[8];
	ss << " , ";
	ss << "\u0022value10\u0022: " << std::setprecision(3) << std::fixed << m_debug_values_student[9];
	ss << "}";
	ss << "}";

	//std::string s = ss.str();
	// Put the string into the response
	//response.data = "test of service for web interface";
	//response.data = "{\"crazyradiostatus\": \"%d\"}", m_crazyradio_status;
	response.data = ss.str();

	// Return success
	return true;
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
	ros::init(argc, argv, "AgentStatusForWebInterface");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "AgentStatusForWebInterface" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[AGENT STATUS FOR WEB INTERFACE] ros::this_node::getNamespace() =  " << m_namespace);





	// SUBSCRIBERS

	// FOR THE CRAZY RADIO STATUS:
	// > Get a node handle to the Crazy Radio
	std::string namespace_to_crazyradio = m_namespace + "/CrazyRadio";
	ros::NodeHandle nodeHandle_to_crazyradio(namespace_to_crazyradio);
	// > Subscribe to the topic
	ros::Subscriber crazyRadioStatusSubscriber = nodeHandle_to_crazyradio.subscribe("CrazyRadioStatus", 1, crazyRadioStatusCallback);

	// FOR THE BATTERY LEVEL
	// > Get a node handle to the Battery Monitor
	std::string namespace_to_BatteryMonitor = m_namespace + "/BatteryMonitor";
	ros::NodeHandle nodeHandle_to_BatteryMonitor(namespace_to_BatteryMonitor);
	// > Subscribe to the topic
	ros::Subscriber newBatteryLevelSubscriber = nodeHandle_to_BatteryMonitor.subscribe("Level", 1, newBatteryLevelCallback);

	// FOR THE FLYING STATE OF THE AGENT
	// > Get a node handle to the Flying Agent Client
	std::string namespace_to_FlyingAgentClient = m_namespace + "/FlyingAgentClient";
	ros::NodeHandle nodeHandle_to_FlyingAgentClient(namespace_to_FlyingAgentClient);
	// > Subscribe to the topic
	ros::Subscriber agentOperatingStateSubscriber = nodeHandle_to_FlyingAgentClient.subscribe("FlyingState", 1, agentOperatingStateCallback);

	// FOR THE INSTANT CONTROLLER
	// > Subscribe to the topic
	ros::Subscriber instantControllerSubscriber = nodeHandle_to_FlyingAgentClient.subscribe("ControllerUsed", 1, instantControllerChangedCallback);

	// FOR THE DEFAULT CONTROLLER SETPOINT
	// > Get a node handle to the Default Controller Service
	std::string namespace_to_DefaultController = m_namespace + "/DefaultControllerService";
	ros::NodeHandle nodeHandle_to_DefaultController(namespace_to_DefaultController);
	// > Subscribe to the topic
	ros::Subscriber defaultControllerSetpointChangedSubscriber = nodeHandle_to_DefaultController.subscribe("SetpointChanged", 1, defaultControllerSetpointChangedCallback);

	// FOR THE STUDENT CONTROLLER SETPOINT
	// > Get a node handle to the Student Controller Service
	std::string namespace_to_StudentController = m_namespace + "/StudentControllerService";
	ros::NodeHandle nodeHandle_to_StudentController(namespace_to_StudentController);
	// > Subscribe to the topic
	ros::Subscriber studentControllerSetpointChangedSubscriber = nodeHandle_to_StudentController.subscribe("SetpointChanged", 1, studentControllerSetpointChangedCallback);

	// FOR THE DEBUG VALUES OF THE STUDENT CONTROLLER
	// > Subscribe to the topic
	ros::Subscriber studentControllerDebugValuesSubscriber = nodeHandle_to_StudentController.subscribe("DebugTopic", 1, studentControllerDebugValuesCallback);

	// FOR THE STATE ESTIMATE FROM THE CRAZYFLIE
	// > Get a node handle to the Crazy Radio node
	std::string namespace_to_CrazyRadio = m_namespace + "/CrazyRadio";
	ros::NodeHandle nodeHandle_to_CrazyRadio(namespace_to_CrazyRadio);
	// > Subscribe to the topic
	ros::Subscriber cfStateEstimateSubscriber = nodeHandle_to_CrazyRadio.subscribe("CFStateEstimate", 1, cfStateEstimateCallback);





	// SERVICE SERVERS

	// Service callback for provide status to the Web Interface
	ros::ServiceServer statusForWebInterfaceService = nodeHandle.advertiseService("StatusAsJson", statusForWebInterfaceCallback);





	// Inform the user the this node is ready
	ROS_INFO("[AGENT STATUS FOR WEB INTERFACE] Ready :-)");
	// Spin as a single-thread node
	ros::spin();

	return 0;
}
