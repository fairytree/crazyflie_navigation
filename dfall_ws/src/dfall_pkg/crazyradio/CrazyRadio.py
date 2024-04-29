#!/usr/bin/env python
# -*- coding: utf-8 -*-

#    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
#
#    This file is part of D-FaLL-System.
#    
#    D-FaLL-System is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#    
#    D-FaLL-System is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    
#    You should have received a copy of the GNU General Public License
#    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
#    
#
#    ----------------------------------------------------------------------------------
#    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
#    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
#    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
#    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
#    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
#
#
#    DESCRIPTION:
#    The service that manages the context of the student groups.
#
#    ----------------------------------------------------------------------------------


# Import the ROS library package
import roslib
# Load the "dfall package" manifest for ROS
roslib.load_manifest('dfall_pkg')
# Import the ROS-Python package
import rospy
# Import standard messages
from std_msgs.msg  import Int32
# Import dfall_pkg messages
from dfall_pkg.msg import ControlCommand
from dfall_pkg.msg import IntWithHeader
from dfall_pkg.msg import FlyingVehicleState
#from dfall_pkg.msg import GyroMeasurements
# Import dfall_pkg services
from dfall_pkg.srv import IntIntService

# General import
import time, sys
import struct
import logging

# Import the ROS bag package for logging
#import rosbag
#from rospkg import RosPack

# Import the ROS standard message types
from std_msgs.msg import Float32
from std_msgs.msg import String

# Add library
#sys.path.append("lib")

# CrazyFlie client imports
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

import cflib.drivers.crazyradio

# Import the package for logging data back from the Crazyflie
from cflib.crazyflie.log import LogConfig

# Logging settings
logging.basicConfig(level=logging.ERROR)

# CONTROLLER_MOTOR = 2
# CONTROLLER_ANGLE = 1
# CONTROLLER_RATE = 0

# CF_COMMAND_TYPE_MOTORS =  8
# CF_COMMAND_TYPE_RATE   =  9
# CF_COMMAND_TYPE_ANGLE  = 10

CF_ONBOARD_CONTROLLER_MODE_OFF            = 0
CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE   = 1
CF_ONBOARD_CONTROLLER_MODE_ANGLE          = 2
CF_ONBOARD_CONTROLLER_MODE_VELOCITY       = 3
CF_ONBOARD_CONTROLLER_MODE_POSITION       = 4

# CF_PACKET_DECODER_STOP_TYPE              = 0
# CF_PACKET_DECODER_VELOCITY_WORLD_TYPE    = 1
# CF_PACKET_DECODER_Z_DISTANCE_TYEP        = 2
# CF_PACKET_DECODER_CPPM_EMU_TYPE          = 3
# CF_PACKET_DECODER_ALT_HOLD_TYPE          = 4
# CF_PACKET_DECODER_HOVER_TYPE             = 5
# CF_PACKET_DECODER_FULL_STATE_TYPE        = 6
# CF_PACKET_DECODER_POSITION_TYPE          = 7
CF_PACKET_DECODER_DFALL_TYPE             = 8

RAD_TO_DEG = 57.296

# CrazyRadio states:
CONNECTED    = 0
CONNECTING   = 1
DISCONNECTED = 2

# Commands coming
CMD_RECONNECT  = 0
CMD_DISCONNECT = 1

# Commands for FlyingAgentClient
#CMD_USE_SAFE_CONTROLLER    = 1
#CMD_USE_DEMO_CONTROLLER    = 2
#CMD_USE_STUDENT_CONTROLLER = 3
#CMD_USE_MPC_CONTROLLER     = 4
#CMD_USE_REMOTE_CONTROLLER  = 5
#CMD_USE_TUNING_CONTROLLER  = 6

CMD_CRAZYFLY_TAKE_OFF   = 11
CMD_CRAZYFLY_LAND       = 12
CMD_CRAZYFLY_MOTORS_OFF = 13

# Types of flying vehicle states:
#FLYING_VEHICLE_STATE_TYPE_NONE                        = 0
#FLYING_VEHICLE_STATE_TYPE_MOCAP_MEASUREMENT           = 1
FLYING_VEHICLE_STATE_TYPE_CRAZYFLIE_STATE_ESTIMATE    = 2
#FLYING_VEHICLE_STATE_TYPE_12_STATE_ESTIMATE           = 3

# Flying states
STATE_MOTORS_OFF     = 1
#STATE_TAKE_OFF       = 2
#STATE_FLYING         = 3
#STATE_LAND           = 4
#STATE_UNAVAILABLE    = 5



# Open the ROS bag for logging
#rp = RosPack()
#record_file = rp.get_path('dfall_pkg') + '/LoggingOnboard.bag'
#rospy.loginfo('afdsasdfasdfsadfasdfasdfasdfasdfasdfasdf')
#rospy.loginfo(record_file)
#bag = rosbag.Bag(record_file, 'w')

class CrazyRadioClient:
    """
       CrazyRadio client that recieves the commands from the controller and
       sends them in a CRTP package to the crazyflie with the specified
       address.
    """
    def __init__(self):

        # INITIALISE THE PROPERTIES OF THE OBJECT
        # > For the status of the radio link
        self._status = DISCONNECTED
        # > For the addess of the radio link
        self.link_uri = ""
        # > For the name of the connected crazyflie
        self.crazyflie_name = ""
        # > For keeping track of the "flying state"
        #   of the "FlyingAgentClient"
        self._flyingState_of_flyingAgentClient = STATE_MOTORS_OFF

        # > INIIALISE PUBLISHERS
        # > For informing the network of the status of
        #   the radio link
        self.radio_status_publisher = rospy.Publisher(node_name + '/CrazyRadioStatus', Int32, queue_size=1)
        # > For publishing commands on behalf of the
        #   FlyingAgentClient, only used for publishing
        #   "motor off" commands
        self.flyingAgentClient_command_publisher = rospy.Publisher('FlyingAgentClient/Command', IntWithHeader, queue_size=1)

        # PAUSE SHORTLY FOR THE PUBLISHERS TO BE REGISTERED IN ROS
        time.sleep(1.0)

        # Initialize the CrazyFlie and add callbacks
        self._init_cf()

        # Connect to the Crazyflie
        self.connect()

    def _init_cf(self):
        self._cf = Crazyflie()

        # Add callbacks that get executed depending on the connection _status.
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.change_status_to(DISCONNECTED)

    def change_status_to(self, new_status):
        print "[CRAZY RADIO] status changed to: %s" % new_status
        self._status = new_status
        self.radio_status_publisher.publish(new_status)

    def get_status(self):
        return self._status

    def update_link_uri(self):
        self.link_uri = "radio://" + rospy.get_param("~crazyflieAddress")

    def update_crazyflie_name(self):
        self.crazyflie_name = rospy.get_param("~crazyflieName")

    def connect(self):
        # update link from ros params
        self.update_link_uri()

        print "[CRAZY RADIO] Connecting to %s" % self.link_uri
        self.change_status_to(CONNECTING)
        rospy.loginfo("[CRAZY RADIO] connecting...")
        self._cf.open_link(self.link_uri)

    def disconnect(self):
        print "[CRAZY RADIO] sending Motors OFF command before disconnecting"
        cf_client._send_to_commander(0, 0, 0, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0)
        # change state to motors OFF
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        self.flyingAgentClient_command_publisher.publish(msg)
        time.sleep(0.1)
        print "[CRAZY RADIO] Disconnecting from %s" % self.link_uri
        self._cf.close_link()
        self.change_status_to(DISCONNECTED)



    def _data_received_battery_callback(self, timestamp, data, logconf):
        # Initialise the variable for the data
        batteryVolt = Float32()
        # Retrieve the values from the data packet received
        batteryVolt.data = data["pm.vbat"]

        # Write data to the ROS bag
        #bag.write('batteryVoltage', batteryVolt)

        # Print out one value for DEBUGGING
        #print "[CRAZY RADIO] DEBUGGING, received pm.vbat = %f" % batteryVolt.data

        # Publish the battery voltage
        cfbattery_pub.publish(batteryVolt)



    def _data_received_stateEstimate_callback(self, timestamp, data, logconf):
        # Perform safety check if required
        if (isEnabled_strictSafety):
            # Estimate the height at the next measurement
            height_at_next_measurement = data["stateEstimateZ.z"] / 1000.0 + (cfStateEstimate_polling_period / 1000.0) * (data["stateEstimateZ.vz"] / 1000.0)
            # Turn-off if too high
            if (height_at_next_measurement > maxHeight_for_strictSafety_meters):
                # Publish a motors OFF command
                msg = IntWithHeader()
                msg.shouldCheckForAgentID = False
                msg.data = CMD_CRAZYFLY_MOTORS_OFF
                self.flyingAgentClient_command_publisher.publish(msg)
                # Inform the user
                rospy.logerr("[CRAZY RADIO] Height safety check failed, measured = %f, max allowed = %f" % (height_at_next_measurement, maxHeight_for_strictSafety_meters))

        # Initialise the variable for the flying vehicle state
        cfStateEstimate = FlyingVehicleState()

        # Retrieve the values from the data packet received
        # > (x,y,z) positions
        cfStateEstimate.x  = data["stateEstimateZ.x"] / 1000.0
        cfStateEstimate.y  = data["stateEstimateZ.y"] / 1000.0
        cfStateEstimate.z  = data["stateEstimateZ.z"] / 1000.0
        # > (x,y,z) velocities
        cfStateEstimate.vx = data["stateEstimateZ.vx"] / 1000.0
        cfStateEstimate.vy = data["stateEstimateZ.vy"] / 1000.0
        cfStateEstimate.vz = data["stateEstimateZ.vz"] / 1000.0
        # > (x,y,z) accelerations
        #cfStateEstimate.ax = data["stateEstimateZ.ax"] / 1000.0
        #cfStateEstimate.ay = data["stateEstimateZ.ay"] / 1000.0
        #cfStateEstimate.az = data["stateEstimateZ.az"] / 1000.0
        # > (roll,pitch,yaw) angles
        cfStateEstimate.roll  =  data["stateEstimateZ.roll"]  / 1000.0
        cfStateEstimate.pitch = -data["stateEstimateZ.pitch"] / 1000.0
        cfStateEstimate.yaw   =  data["stateEstimateZ.yaw"]   / 1000.0
        # > (roll,pitch,yaw) anglular rates (direct from gryo)
        #cfStateEstimate.rollRate  =  data["stateEstimateZ.rateRoll"]  / 1000.0
        #cfStateEstimate.pitchRate = -data["stateEstimateZ.ratePitch"] / 1000.0
        #cfStateEstimate.yawRate   =  data["stateEstimateZ.rateYaw"]   / 1000.0

        # Print out one value for DEBUGGING
        #print "[CRAZY RADIO] received (z,vz)  = ( %6.3f , %6.3f )" %( cfStateEstimate.z , cfStateEstimate.vz )
        #print "[CRAZY RADIO] received (r,p,y) = ( %6.3f , %6.3f , %6.3f )" %( cfStateEstimate.roll*RAD_TO_DEG , cfStateEstimate.pitch*RAD_TO_DEG , cfStateEstimate.yaw*RAD_TO_DEG )

        # Fill in the remaining details:
        # > For the type
        cfStateEstimate.type = FLYING_VEHICLE_STATE_TYPE_CRAZYFLIE_STATE_ESTIMATE
        # > For the name
        cfStateEstimate.vehicleName = self.crazyflie_name
        # > For the validity flag
        cfStateEstimate.isValid = True
        # > For the acquiring time
        cfStateEstimate.acquiringTime = 0.0

        # Publish the state estimate
        cfstateEstimate_pub.publish(cfStateEstimate)



        # # Initialise the variable for the gyroscope data
        # gyroMeasurements = GyroMeasurements()

        # # Retrieve the values from the data packet received
        # # > (roll,pitch,yaw) anglular rates (direct from gryo)
        # gyroMeasurements.rollrate  =  data["stateEstimateZ.rateRoll"] / 1000.0
        # gyroMeasurements.pitchrate = -data["stateEstimateZ.ratePitch"] / 1000.0
        # gyroMeasurements.yawrate   =  data["stateEstimateZ.rateYaw"] / 1000.0

        # # Fill in the name
        # gyroMeasurements.vehicleName = self.crazyflie_name

        # # Print out one value for DEBUGGING
        # #print "[CRAZY RADIO] gyro (r,p,y)     = ( %6.3f , %6.3f , %6.3f )" %( gyroMeasurements.rollrate*RAD_TO_DEG , gyroMeasurements.pitchrate*RAD_TO_DEG , gyroMeasurements.yawrate*RAD_TO_DEG )
        
        # # Publish the gyroscope measurements
        # cfgyroMeasurements_pub.publish(gyroMeasurements)



    def _logging_error(self, logconf, msg):
        print "[CRAZY RADIO] Error when logging %s" % logconf.name



    def _start_logging(self):

        # NOTE: that logging can only be started when
        # connected to a Crazyflie is becuse "add_config(...)"
        # function check that the variables requested are
        # available in the onboard TOC (table of contents).

        # A "LOGGING GROUP" FOR THE BATTERY VOLTAGE:

        # Initialise a log config object
        self.logconf_battery = LogConfig("LoggingBattery", period_in_ms=battery_polling_period)
        # Add the varaibles to be logged
        self.logconf_battery.add_variable("pm.vbat", "float");
        #
        # Try to add the log config to the crazyflie object
        try:
            self._cf.log.add_config(self.logconf_battery)
            # Add the callback for received data
            self.logconf_battery.data_received_cb.add_callback(self._data_received_battery_callback)
            # Add the callback for errors
            self.logconf_battery.error_cb.add_callback(self._logging_error)
            # Check the log config is valid
            if self.logconf_battery.valid:
                # Start the logging
                self.logconf_battery.start()
                rospy.loginfo("[CRAZY RADIO] Started the log config for the battery voltage")
            else:
                rospy.loginfo("[CRAZY RADIO] The log config for the battery voltage is invalid, hence logging has not been started.")
        # Handle "key error" exceptions:
        except KeyError as e:
            rospy.logerr("[CRAZY RADIO] For the battery voltage, the following error occurred while trying to add the log config: %s" % str(e) ) 
        # Handle "attribution error" exceptions:
        except AttributeError:
            rospy.logerr("[CRAZY RADIO] The battery voltage log config has a bad configuration.")

        # A "LOGGING GROUP" FOR THE STATE ESTIMATE:

        # Initialise a log config object
        self.logconf_stateEstimate = LogConfig("LoggingStateEstimate", period_in_ms=cfStateEstimate_polling_period)
        # Add the varaibles to be logged
        # > (x,y,z) positions
        self.logconf_stateEstimate.add_variable("stateEstimateZ.x",    "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.y",    "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.z",    "int16_t");
        # > (x,y,z) velocities
        self.logconf_stateEstimate.add_variable("stateEstimateZ.vx",   "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.vy",   "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.vz",   "int16_t");
        # > (x,y,z) accelerations
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.ax",   "int16_t");
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.ay",   "int16_t");
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.az",   "int16_t");
        # > (roll,pitch,yaw) angles
        self.logconf_stateEstimate.add_variable("stateEstimateZ.roll",        "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.pitch",       "int16_t");
        self.logconf_stateEstimate.add_variable("stateEstimateZ.yaw",         "int16_t");
        # > (roll,pitch,yaw) anglular rates (direct from gryo)
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.rateRoll",    "int16_t");
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.ratePitch",   "int16_t");
        #self.logconf_stateEstimate.add_variable("stateEstimateZ.rateYaw",     "int16_t");
        #
        # Try to add the log config to the crazyflie object
        try:
            self._cf.log.add_config(self.logconf_stateEstimate)
            # Add the callback for received data
            self.logconf_stateEstimate.data_received_cb.add_callback(self._data_received_stateEstimate_callback)
            # Add the callback for errors
            self.logconf_stateEstimate.error_cb.add_callback(self._logging_error)
            # Check the log config is valid
            if self.logconf_stateEstimate.valid:
                # Start the logging
                self.logconf_stateEstimate.start()
                rospy.loginfo("[CRAZY RADIO] Started the log config for the state estimate")
            else:
                rospy.loginfo("[CRAZY RADIO] The log config for the state estimate is invalid, hence logging has not been started.")
        # Handle "key error" exceptions:
        except KeyError as e:
            rospy.logerr("[CRAZY RADIO] For the state estimate, the following error occurred while trying to add the log config: %s" % str(e) ) 
        # Handle "attribution error" exceptions:
        except AttributeError:
            rospy.logerr("[CRAZY RADIO] The state estimate log config has a bad configuration.")



    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        self.change_status_to(CONNECTED)
        # change state to motors OFF
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        cf_client.flyingAgentClient_command_publisher.publish(msg)

        rospy.loginfo("[CRAZY RADIO] Connection to %s successful: " % link_uri)
        # Config for Logging
        self._start_logging()



    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("[CRAZY RADIO] Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("[CRAZY RADIO] Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.change_status_to(DISCONNECTED)
        rospy.logwarn("[CRAZY RADIO] Disconnected from %s" % link_uri)

        # CHANGE THE STATE TO "MOTORS OFF"
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        self.flyingAgentClient_command_publisher.publish(msg)

        # STOP AND DELETE ALL THE "LOGGING GROUPS"
        # > For the battery voltage
        self.logconf_battery.stop()
        rospy.loginfo("The log config for the battery voltage was stopped")
        self.logconf_battery.delete()
        rospy.loginfo("The log config for the battery voltage was deleted")
        # > For the state estimate
        self.logconf_stateEstimate.stop()
        rospy.loginfo("The log config for the state estimate was stopped")
        self.logconf_stateEstimate.delete()
        rospy.loginfo("The log config for the state estimate was deleted")



    def _send_to_commander(self, cmd1, cmd2, cmd3, cmd4, x_mode , x_value, y_mode, y_value, z_mode, z_value, yaw_mode, yaw_value ):
        # CONSTRUCT THE CONDENSED 16-BIT INTEGER THAT ENCODES
        # THE REQUESTED {x,y,z,yaw} MODES
        # > Each mode is encoded using 3 bits, stacked together as follows:
        # ---------------------------------------------------------------
        # |BIT   | 15 14 13 12 | 11 10  9 | 8  7  6 | 5  4  3 | 2  1  0 |
        # |DESC. | free bits   | yaw_mode | z_mode  | y_mode  | x_mode  |
        # ---------------------------------------------------------------
        # > The python bitwise operators "<<" and "|" are used
        #   to construct the 16-bit "cumulative" mode
        modes_as_uint16 = (yaw_mode << 9) | (z_mode << 6) | (y_mode << 3) | (x_mode << 0)
        #
        # > To test the behaviour of this command, enter the following
        #   into a python shell:
        # >>> modes = (4 << 9) | (2 << 6) | (1 << 3) | (7 << 0)
        # >>> "{0:b}".format(modes)
        # > The second line should display the result '100010001111'
        #   which encodes:
        # --------------------------------------------------
        # |BIT    | 11 10  9 | 8  7  6 | 5  4  3 | 2  1  0 |
        # |O or 1 |  1  0  0 | 0  1  0 | 0  0  1 | 1  1  1 |
        # |Value  | mode=4   | mode=2  | mode=1  | mode=7  |
        # --------------------------------------------------
        #
        # > To extract a mode, an example code snippet is:
        # >>> yaw_mode = (modes >> 9) & 7
        # > The bitwise & operator with the number 7 serves to
        #   select only the right-hand most 3 bits of (modes >> 9),
        #   the last 3 bits of the variable modes after is has been
        #   shifted to the right by 9 bits.
        # > This syntax works in both python and plain-c

        

        # CONSTRUCT AND SEND THE COMMANDER PACKET
        # > See hints at end of this file for an explanation
        #   of the struct packing format, i.e., of '<HHHHHffff'
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHHffff', CF_PACKET_DECODER_DFALL_TYPE, modes_as_uint16, cmd1, cmd2, cmd3, cmd4, x_value, y_value, z_value, yaw_value)
        self._cf.send_packet(pk)

        # NOTES ON THE PACKING OF THE "pk.data" STRUCT
        # > The "B" is the decoder type packed into 1 btye
        # > The first "H" is the condense 2-byte integer
        #   of four onboard controller modes
        # > The next four "H" are the 2-byte per motor commands
        # > The four "f" are the setpoints for each onboard controller
        # > Hence the total size of the data is:
        #   sizs('HHHHHffff') = 5*2bytes + 4*4bytes = 26bytes
        # > This leaves 3 spare bytes before reaching the maximum
        #   of 29 bytes as specified in the comment below

        # DESCRIPTION OF THE CRAZYFLIE PACKETS
        # > This is taken directly from the file
        #   "crtp_commander_generic.c" in the
        #   Crazyflie firmware
        #
        # The generic commander format contains a packet type and data that has to be
        # decoded into a setpoint_t structure. The aim is to make it future-proof
        # by easily allowing the addition of new packets for future use cases.
        #
        # The packet format is:
        # +------+==========================+
        # | TYPE |     DATA                 |
        # +------+==========================+
        #
        # The type is defined bellow together with a decoder function that should take
        # the data buffer in and fill up a setpoint_t structure.
        # The maximum data size is 29 bytes.





    def crazyRadioCommandCallback(self, msg):
        """Callback to tell CrazyRadio to reconnect"""

        # Initialise a boolean flag that the command is NOT relevant
        command_is_relevant = False

        # Check the header details of the message for it relevance
        if (msg.shouldCheckForAgentID == False):
            command_is_relevant = True
        else:
            for this_ID in msg.agentIDs:
                if (this_ID == m_agentID):
                    command_is_relevant = True
                    break

        # Only consider the command if it is relevant
        if (command_is_relevant):
            #print "[CRAZY RADIO] received command to: %s" % msg.data
            if msg.data == CMD_RECONNECT:
                if self.get_status() == DISCONNECTED:
                    print "[CRAZY RADIO] received command to CONNECT (current status is DISCONNECTED)"
                    self.connect()
                elif self.get_status() == CONNECTING:
                    print "[CRAZY RADIO] received command to CONNECT (current status is CONNECTING)"
                    #self.radio_status_publisher.publish(CONNECTING)
                elif self.get_status() == CONNECTED:
                    print "[CRAZY RADIO] received command to CONNECT (current status is CONNECTED)"
                    #self.radio_status_publisher.publish(CONNECTED)

            elif msg.data == CMD_DISCONNECT:
                if self.get_status() == CONNECTED:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is CONNECTED)"
                    self.disconnect()
                elif self.get_status() == CONNECTING:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is CONNECTING)"
                    #self.radio_status_publisher.publish(CONNECTING)
                elif self.get_status() == DISCONNECTED:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is DISCONNECTED)"
                    #self.radio_status_publisher.publish(DISCONNECTED)



    def getCurrentCrazyRadioStatusServiceCallback(self, req):
        """Callback to service the request for the connection status"""
        # Directly return the current status
        return self._status



    def flyingStateCallback(self, msg):
        """Callback for keeping track of the flying state"""

        # > This is used to know when to reset the Crazyflie's
        #   onboard Kalman filter.
        # > The filter is reset when the state changes from
        #   "motors off" to something else
        # > This is required because the onboard state estimate
        #   tends to drift when the Crazyflie stays on the ground
        #   for an even a short amount of time

        # The data in this message is always relevant because
        # of the namespace of the message, hence there is no
        # need to check the header.

        # Get the new state from the message
        new_flying_state = msg.data

        # Get the current flying state into a local variable
        current_flying_state = self._flyingState_of_flyingAgentClient

        # Only consider the command if it is relevant
        if ( \
            (current_flying_state==STATE_MOTORS_OFF)
            and \
            not(new_flying_state==STATE_MOTORS_OFF) \
        ):
            # Call the CrazyRadio function that sets the
            # paramter for reseting the onboard estimate
            self._cf.param.set_value("kalman.resetEstimation", 1)
            # Inform the user
            #print "[CRAZY RADIO] reqested the Crazyflie to reset the onboard estimate"

        # Update the class variable for keeping track of the
        # current state
        self._flyingState_of_flyingAgentClient = new_flying_state


# END OF: class CrazyRadioClient:
# ============================= #


# ============================= #
# CALL BACK WHEN A MESSAGE IS RECEIVED ON:
# "FlyingAgentClient/ControlCommand"
def controlCommandCallback(data):
    """Callback for controller actions"""

    # NOTE: cmd{1,...,4} must NOT be all 0, as this causes the
    #       crazyflie onboard controller to reset!
    # TODO: check if this note is still true!

    # CHECK THE X-CONTROLLER MODE
    # > Because the pitch is inverted on the crazyflie
    # > And because the angles need to be converted to degrees
    if ( \
        (data.xControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE) \
        or \
        (data.xControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGLE) \
    ):
        # Negate the setpoint and convert to degrees
        x_controller_setpoint = -data.xControllerSetpoint * RAD_TO_DEG;
    #
    else:
        # Take the setpoint as is
        x_controller_setpoint = data.xControllerSetpoint;

    # CHECK THE Y-CONTROLLER MODE
    # > Because the pitch is inverted on the crazyflie
    # > And because the angles need to be converted to degrees
    if ( \
        (data.yControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE) \
        or \
        (data.yControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGLE) \
    ):
        # Convert the setpoint to degrees
        y_controller_setpoint = data.yControllerSetpoint * RAD_TO_DEG;
    #
    else:
        # Take the setpoint as is
        y_controller_setpoint = data.yControllerSetpoint;

    # CHECK THE Z-CONTROLLER MODE
    # > Because the angle and angular rate modes are not allowed
    if ( \
        (data.zControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE) \
        or \
        (data.zControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGLE) \
    ):
        # Turn the z-controller off and inform the user
        rospy.logerr("[CRAZY RADIO] z controller requested disallowed mode, now turning OFF the z-controller.")
        z_controller_mode     = CF_ONBOARD_CONTROLLER_MODE_OFF;
        z_controller_setpoint = 0.0;
    #
    else:
        # Take the mode and setpoint as is
        z_controller_mode     = data.zControllerMode;
        z_controller_setpoint = data.zControllerSetpoint;

    # CHECK THE YAW-CONTROLLER MODE
    # > Because the yaw is inverted on the crazyflie
    # > And because the angles need to be converted to degrees
    # > And because the position and velocity modes are not allowed
    if ( \
        (data.yawControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE) \
        or \
        (data.yawControllerMode==CF_ONBOARD_CONTROLLER_MODE_ANGLE) \
    ):
        # Negate the setpoint and convert to degrees
        yaw_controller_mode     = data.yawControllerMode;
        yaw_controller_setpoint = data.yawControllerSetpoint * RAD_TO_DEG;
    #
    elif (data.yawControllerMode==CF_ONBOARD_CONTROLLER_MODE_OFF):
        # Set the setpoint to zero
        yaw_controller_mode     = data.yawControllerMode;
        yaw_controller_setpoint = 0.0;
    #
    else:
        # Set the yaw-controller  to zero angle setpoint and inform the user
        rospy.logerr("[CRAZY RADIO] yaw controller requested disallowed mode, now turning to ANGULAR_RATE mode with zero setpoint.")
        yaw_controller_mode     = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
        yaw_controller_setpoint = 0.0;


    # PASS THE COMMAND TO THE SENDING FUNCTION
    cf_client._send_to_commander(
        data.motorCmd1,
        data.motorCmd2,
        data.motorCmd3,
        data.motorCmd4,
        data.xControllerMode,
        x_controller_setpoint,
        data.yControllerMode,
        y_controller_setpoint,
        z_controller_mode,
        z_controller_setpoint,
        yaw_controller_mode,
        yaw_controller_setpoint,
        )

# END OF: def controlCommandCallback(data):
# ============================= #





if __name__ == '__main__':

    # Starting the ROS-node
    global node_name
    node_name = "CrazyRadio"
    rospy.init_node(node_name, anonymous=True)

    # Get the namespace of this node
    global ros_namespace
    ros_namespace = rospy.get_namespace()



    # Initialize the low-level drivers
    # > with the option set to NOT list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Wait until address parameter is set by FlyingAgentClient
    while not rospy.has_param("~crazyflieAddress"):
        time.sleep(0.05)

    # Wait until name parameter is set by FlyingAgentClient
    # > Should be set at the same timie as the address
    while not rospy.has_param("~crazyflieName"):
        time.sleep(0.05)

    # Use the following lines of code to connect without data
    # from the "Central Manager":
    # radio_address = "radio://0/72/2M"
    # rospy.loginfo("manual address loaded")

    # Fetch the YAML paramter "battery polling period"
    global battery_polling_period
    battery_polling_period = rospy.get_param(ros_namespace + "CrazyRadio/CrazyRadioCopyOfBatteryMonitor/battery_polling_period")

    global cfStateEstimate_polling_period
    cfStateEstimate_polling_period = rospy.get_param(ros_namespace + "CrazyRadio/CrazyRadioConfig/cfStateEstimate_polling_period")

    global isEnabled_strictSafety
    isEnabled_strictSafety = rospy.get_param(ros_namespace + "CrazyRadio/CrazyRadioConfig/isEnabled_strictSafety")

    global maxHeight_for_strictSafety_meters
    maxHeight_for_strictSafety_meters = rospy.get_param(ros_namespace + "CrazyRadio/CrazyRadioConfig/maxHeight_for_strictSafety_meters")

    # Fetch the YAML paramter "agentID" and "coordID"
    global m_agentID
    m_agentID = rospy.get_param(ros_namespace + "/FlyingAgentClient/agentID")
    coordID   = rospy.get_param(ros_namespace + "/FlyingAgentClient/coordID")
    # Convert the coordinator ID to a zero-padded string
    coordID_as_string = format(coordID, '03')

    # Initialise a publisher for the battery voltage
    global cfbattery_pub
    cfbattery_pub = rospy.Publisher(node_name + '/CFBattery', Float32, queue_size=10)

    # Initialise a publisher for the state estimate
    global cfstateEstimate_pub
    cfstateEstimate_pub = rospy.Publisher(node_name + '/CFStateEstimate', FlyingVehicleState, queue_size=10)

    # Initialise a "CrazyRadioClient" type variable that handles
    # all communication over the CrazyRadio
    global cf_client
    cf_client = CrazyRadioClient()

    # Subscribe to the commands for when to (dis-)connect the
    # CrazyRadio connection with the Crazyflie
    # > For the radio commands from the FlyingAgentClient of this agent
    rospy.Subscriber("FlyingAgentClient/CrazyRadioCommand", IntWithHeader, cf_client.crazyRadioCommandCallback)
    # > For the radio command from the coordinator
    rospy.Subscriber("/dfall/coord" + coordID_as_string + "/FlyingAgentClient/CrazyRadioCommand", IntWithHeader, cf_client.crazyRadioCommandCallback)


    # Advertise a Serice for the current status
    # of the Crazy Radio connect
    getCurrentCrazyRadioStatusService = rospy.Service(node_name + "/getCurrentCrazyRadioStatus", IntIntService , cf_client.getCurrentCrazyRadioStatusServiceCallback)


    # Subscribe to the "flying state" of the FlyingAgentClient
    # > This is used to determine when to reset the Crazyflie's
    #   onboard Kalman filter
    rospy.Subscriber("FlyingAgentClient/FlyingState", Int32, cf_client.flyingStateCallback)

    # Sleep briefly to allow everything to start-up
    time.sleep(1.0)



    # Subscribe to the "control commands" that are to be sent
    # to the Crazyflie, i.e., commands of motor thrust and
    # setpoints for the onboard controllers
    rospy.Subscriber("FlyingAgentClient/ControlCommand", ControlCommand, controlCommandCallback)



    # Print out some information to the user.
    print "[CRAZY RADIO] Node READY :-)"

    # Enter an endless while loop to keep the node alive.
    rospy.spin()



    # SHUT DOWN THE NODE IF THE "ros::spin" IS CANCELLED

    # Inform the user
    rospy.loginfo("[CRAZY RADIO] Turning off crazyflie")

    # Send the motors off command
    cf_client._send_to_commander(0, 0, 0, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0, CF_ONBOARD_CONTROLLER_MODE_OFF, 0)

    # Send a "Motors OFF" on behalf of the "FlyingAgentClient"
    # > This serves the purpose of changing the flying state
    #   to the "Motors OFF" state
    msg = IntWithHeader()
    msg.shouldCheckForAgentID = False
    msg.data = CMD_CRAZYFLY_MOTORS_OFF
    cf_client.flyingAgentClient_command_publisher.publish(msg)

    # Allow some time for the command to be sent
    time.sleep(1.0)



    # Close the ROS loggin bag
    #bag.close()
    #rospy.loginfo("[CRAZY RADIO] Logging bag closed")

    # Close the CrazyRadio link
    cf_client._cf.close_link()
    rospy.loginfo("[CRAZY RADIO] Link closed")


# ===================================== #
# HINTS FOR THE FORMAT STRING USED IN
# THE "struct.pack(...)" COMMANDS FOR
# CONSTRUCTING THE DATA PACKAGES SENT
# TO THE CRAZYFLIE
#
# COPIED FROM:
# https://docs.python.org/3/library/struct.html#format-characters
#
# FORMAT CHARACTERS:
# Format characters have the following meaning; the conversion
# between C and Python values should be obvious given their
# types. The ‘Standard size’ column refers to the size of the
# packed value in bytes when using standard size; that is, when
# the format string starts with one of '<', '>', '!' or '='.
# When using native size, the size of the packed value is
# platform-dependent.
#
# |----------------------------------------------------------|
# | Format | C Type             | Python type | Size | Notes |
# |--------------------------------------------------|-------|
# | x      | pad byte           | no value    | 1    |       |
# | c      | char               | bytes len 1 | 1    |       |
# | b      | signed char        | integer     | 1    | (1)(2)|
# | B      | unsigned char      | integer     | 1    | (2)   |
# | ?      | _Bool              | bool        | 1    | (1)   |
# | h      | short              | integer     | 2    | (2)   |
# | H      | unsigned short     | integer     | 2    | (2)   |
# | i      | int                | integer     | 4    | (2)   |
# | I      | unsigned int       | integer     | 4    | (2)   |
# | l      | long               | integer     | 4    | (2)   |
# | L      | unsigned long      | integer     | 4    | (2)   |
# | q      | long long          | integer     | 8    | (2)   |
# | Q      | unsigned long long | integer     | 8    | (2)   |
# | n      | ssize_t            | integer     |      | (3)   |
# | N      | size_t             | integer     |      | (3)   |
# | e      | (6)                | float       | 2    | (4)   |
# | f      | float              | float       | 4    | (4)   |
# | d      | double             | float       | 8    | (4)   |
# | s      | char[]             | bytes       |      |       |
# | p      | char[]             | bytes       |      |       |
# | P      | void *             | integer     |      | (5)   |
# |--------------------------------------------------|-------|
#
# Changed in version 3.3: Added support for the 'n'
# and 'N' formats.
#
# Changed in version 3.6: Added support for the 'e'
# format.
#
# Notes:
#
# (1) The '?' conversion code corresponds to the _Bool
#     type defined by C99. If this type is not available,
#     it is simulated using a char. In standard mode, it
#     is always represented by one byte.
#
# (2) When attempting to pack a non-integer using any of
#     the integer conversion codes, if the non-integer has
#     a __index__() method then that method is called to
#     convert the argument to an integer before packing.
#
#     Changed in version 3.2: Use of the __index__() method
#     for non-integers is new in 3.2.
#
# (3) The 'n' and 'N' conversion codes are only available
#     for the native size (selected as the default or with
#     the '@' byte order character). For the standard size,
#     you can use whichever of the other integer formats
#     fits your application.
#
# (4) For the 'f', 'd' and 'e' conversion codes, the packed
#     representation uses the IEEE 754 binary32, binary64 or
#     binary16 format (for 'f', 'd' or 'e' respectively),
#     regardless of the floating-point format used by the
#     platform.
#
# (5) The 'P' format character is only available for the
#     native byte ordering (selected as the default or with
#     the '@' byte order character). The byte order character
#     '=' chooses to use little- or big-endian ordering based
#     on the host system. The struct module does not interpret
#     this as native ordering, so the 'P' format is not
#     available.
#
# (6) The IEEE 754 binary16 “half precision” type was
#     introduced in the 2008 revision of the IEEE 754 standard.
#     It has a sign bit, a 5-bit exponent and 11-bit precision
#     (with 10 bits explicitly stored), and can represent
#     numbers between approximately 6.1e-05 and 6.5e+04 at
#     full precision. This type is not widely supported by C
#     compilers: on a typical machine, an unsigned short can
#     be used for storage, but not for math operations. See
#     the Wikipedia page on the half-precision floating-point
#     format for more information.
#
# A format character may be preceded by an integral repeat
# count. For example, the format string '4h' means exactly the
# same as 'hhhh'.
#
# Whitespace characters between formats are ignored; a count
# and its format must not contain whitespace though.
#
# For the 's' format character, the count is interpreted as
# the length of the bytes, not a repeat count like for the
# other format characters; for example, '10s' means a single
# 10-byte string, while '10c' means 10 characters. If a count
# is not given, it defaults to 1. For packing, the string is
# truncated or padded with null bytes as appropriate to make
# it fit. For unpacking, the resulting bytes object always has
# exactly the specified number of bytes. As a special case,
# '0s' means a single, empty string (while '0c' means 0
# characters).
#
# When packing a value x using one of the integer formats
# ('b', 'B', 'h', 'H', 'i', 'I', 'l', 'L', 'q', 'Q'), if x is
# outside the valid range for that format then struct.error is
# raised.
#
# Changed in version 3.1: In 3.0, some of the integer formats
# wrapped out-of-range values and raised DeprecationWarning
# instead of struct.error.
#
# The 'p' format character encodes a “Pascal string”, meaning
# a short variable-length string stored in a fixed number of
# bytes, given by the count. The first byte stored is the
# length of the string, or 255, whichever is smaller. The
# bytes of the string follow. If the string passed in to
# pack() is too long (longer than the count minus 1), only
# the leading count-1 bytes of the string are stored. If the
# string is shorter than count-1, it is padded with null bytes
# so that exactly count bytes in all are used. Note that for
# unpack(), the 'p' format character consumes count bytes,
# but that the string returned can never contain more than
# 255 bytes.

# For the '?' format character, the return value is either
# True or False. When packing, the truth value of the argument
# object is used. Either 0 or 1 in the native or standard bool
# representation will be packed, and any non-zero value will
# be True when unpacking.
#
# ===================================== #