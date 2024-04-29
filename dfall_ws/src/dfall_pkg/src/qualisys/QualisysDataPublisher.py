#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# OPTIONS: #!/usr/bin/env python
# OR:      #!/usr/bin/python3
# OR:      #!/usr/bin/python3.5

# WHERE THE KEY INSTALLATIONS TO USE python3 ARE:
# >> sudo apt-get install python3-pip python3-yaml
# >> pip3 install rospkg catkin_pkg --user

# INSTALL THE LATEST VERSION OF QTM
# >> pip3 install qtm --user
#
# OR
#
# For ubuntu 16.04, it is necessary to install an older version of QTM
# because of compatibility with Python 3.5.2 versus >3.5.3, the commit
# key from the GitHub repository is:
# qualisys_python_sdk-327bf90e2829e02c40d3d85ed11e6e4d2d25f8a8
#
# Once downloaded, this can be installed from the local files using:
# pip3 install -e /path/to/folder/containing/setup.py
#
# Install also the requirements
# Note: that you may need to remove "black==18.6b4" from the list
# in the "requirements.txt" file pip3 throws an error
# >> pip3 install -r /path/to/requirements.txt

#    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
#    The publisher for the Qualisys motion capture data
#
#    ----------------------------------------------------------------------------------



# Import ros and the D-FaLL package
import roslib; roslib.load_manifest('dfall_pkg')
import rospy

# Import the standard message types
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String

# Import the D-FaLL message types
from dfall_pkg.msg import ViconData
from dfall_pkg.msg import FlyingVehicleState
from dfall_pkg.msg import UnlabeledMarker

# General import
import time
import math

# Not sure if any of these are required
#import logging
#import datetime
#from enum import Enum
#import sys
#import struct

# ROS imports
#import rosbag
from rospkg import RosPack

# Import the library for asynchronous I/O
import asyncio

# Import the library for parsing XML trees
import xml.etree.ElementTree as ET

# Import the Qualisys Track Manager (QTM) Library
import qtm





# DEFINE CONSTANTS

# Conversion between radians and degrees
RAD_TO_DEG = 57.296
DEG_TO_RAD = 0.01745329

# Path to the file with "fake" data for testing QTM
#QTM_FILE = pkg_resources.resource_filename("qtm", "data/Demo.qtm")
QTM_FILE = "none"

# Types of flying vehicle states:
#FLYING_VEHICLE_STATE_TYPE_NONE                        = 0
FLYING_VEHICLE_STATE_TYPE_MOCAP_MEASUREMENT           = 1
#FLYING_VEHICLE_STATE_TYPE_CRAZYFLIE_STATE_ESTIMATE    = 2
#FLYING_VEHICLE_STATE_TYPE_12_STATE_ESTIMATE           = 3

# OPEN THE ROS BAG FOR WRITING
#rp = RosPack()
#record_file = rp.get_path('dfall_pkg') + '/LoggingOnboard.bag'
#rospy.loginfo('afdsasdfasdfsadfasdfasdfasdfasdfasdfasdf')
#rospy.loginfo(record_file)
#bag = rosbag.Bag(record_file, 'w')





class QualisysQtmClient:
    """
       Qualisys Data Publisher that connects to the
       Qualisys Track Manager (QTM) server, receives
       the 6DOF data, and publishs it for other nodes
       to use.
    """
    def __init__(self):

        # QTM CONNECTION STATUS AND OBJECT
        self.qtm_ip = "10.42.00.15:801"
        self._qtm_status = "empty"
        self._qtm_connection = None

        self._body_to_index_dictionary = None
        self._list_of_body_names = None

        self._qualisys_data_publisher = rospy.Publisher(node_name + '/ViconData', ViconData, queue_size=1)


        # WAIT FOR A SHORT TIME BEFORE PROCEEDING WITH INITIALISATION
        time.sleep(0.1)

        # Initialize the CrazyFlie and add callbacks
        self._init_qtm_client()





    # INITIALISE THE QTM CLIENT OBJECT
    def _init_qtm_client(self):
        # Get the "qtm_host_ip_address" parameter that is the IP address
        self.qtm_ip = rospy.get_param("~qtm_host_ip_address")


    # CONNECT TO THE QTM SERVER
    async def connect_to_qtm(self):
        # Inform the user
        print("[QUALISYS DATA PUBLISHER] now attempting to connect to QTM at IP address ", self.qtm_ip )

        # Connect to QTM
        self._qtm_connection = await qtm.connect( self.qtm_ip )

        # Inform the user is the QTM connection failed
        if self._qtm_connection is None:
            print("[QUALISYS DATA PUBLISHER] connection is NONE for IP address = " , self.qtm_ip )
            return

        # Take control of qtm, context manager will automatically release control after scope end
        async with qtm.TakeControl(self._qtm_connection, "password"):

            # Specify whether to:
            # True  = get real-time QTM
            # False = get data from file
            get_realtime_data = True

            if get_realtime_data:
                
                # Get the current connection state to the QTM server
                current_connection_state = await self._qtm_connection.get_state()

                # Inform the user of the current connection state
                print( "[QUALISYS DATA PUBLISHER] QTM connection State = " , current_connection_state )

                # If not currently connected to the QTM serever...
                if (current_connection_state != qtm.QRTEvent.EventConnected):
                    # ... then start new connection to the QTM server
                    await self._qtm_connection.new()

            else:
                # Load qtm file
                await self._qtm_connection.load(QTM_FILE)

                # start rtfromfile
                await self._qtm_connection.start(rtfromfile=True)

        # Get 6dof settings from QTM
        parameters_6d_as_xml_string = await self._qtm_connection.get_parameters(parameters=["6d"])

        # Convert the XML string to a index of the 6DOF bodies available from QTM
        # This should be a dictionary that has the name of the object, and its index,
        # for example: {'CF01',0 , 'CF02',1}
        self._body_to_index_dictionary = self.create_body_index(parameters_6d_as_xml_string)

        self._list_of_body_names = self.create_list_of_body_names(parameters_6d_as_xml_string)

        print("[QUALISYS DATA PUBLISHER] Index of 6DOF bodies available from QTM: " , self._body_to_index_dictionary )





    # START STREAMING FRAMES RECIEVED FROM QTM
    async def start_streaming(self):
        # Inform the user
        print("[QUALISYS DATA PUBLISHER] Now starting to stream 6DOF data" )
        # Start streaming frames:
        # > with position and rotation matrix
        #await connection.stream_frames(components=["6d"], on_packet=on_6d_packet)
        # > with position and euler angles
        # Take control of qtm, context manager will automatically release control after scope end
        await self._qtm_connection.stream_frames(frames="allframes",components=["6deuler"], on_packet=self.on_6deuler_packet)

        # Options for .stream_frames:
        # frames=| allframes | frequency:n | frequencydivisor:n |
        # where n should be the desired value




    # STOP STREAMING FRAMES RECIEVED FROM QTM
    async def stop_streaming(self):
        # Stop streaming
        await self._qtm_connection.stream_frames_stop()





    def on_6deuler_packet(self,packet):

        # DEBUGGING: Print the frame number
        #print("[DEBUGGING] Framenumber: {}".format(packet.framenumber))

        # GET THE DATA FROM THE PACKET
        #info, bodies = packet.get_6d()
        info, bodies  = packet.get_6d_euler()

        # DEBUGGING: Print the body count and position of an object
        # print("[DEBUGGING] Body count = ", info.body_count )
        # if info.body_count > 0:
        #     position, rotation = bodies[0]
        #     print( "position.x = " , position[0] )

        if info.body_count > 0:

            # Initilise a "ViconData" struct
            # > This is defined in the "ViconData.msg" file
            viconData = ViconData()

            # Initalise a counter for the bodies
            current_body_index = 0

            # Iterate through the bodies
            for body in bodies:
                
                # Get the position and rotation data of the body
                position, rotation = body

                # DEBUGGING: to check the naming of rotation angles
                #print( "rotation = ", rotation )

                # Initialise a "ViconData" struct
                # > This is defined in the "FlyingVehicleState.msg" file
                this_crazyflie_data = FlyingVehicleState();

                # Add the type
                this_crazyflie_data.type = FLYING_VEHICLE_STATE_TYPE_MOCAP_MEASUREMENT;

                # Add the name of this object
                if current_body_index < len(self._list_of_body_names):
                    this_crazyflie_data.vehicleName = self._list_of_body_names[current_body_index]
                else:
                    this_crazyflie_data.vehicleName = ''

                # Check for "nan" as an indication of occulsion
                if math.isnan(position.x):
                    # Fill in the occlusion flag
                    this_crazyflie_data.isValid = False
                else:
                    # Fill in the occlusion flag
                    this_crazyflie_data.isValid = True

                    # Fill in the position data
                    this_crazyflie_data.x = position.x * 0.001
                    this_crazyflie_data.y = position.y * 0.001
                    this_crazyflie_data.z = position.z * 0.001

                    # Fill in the position data
                    this_crazyflie_data.roll  = rotation.a3 * DEG_TO_RAD
                    this_crazyflie_data.pitch = rotation.a2 * DEG_TO_RAD
                    this_crazyflie_data.yaw   = rotation.a1 * DEG_TO_RAD

                # DEBUGGIN: print out the crazyflie data
                #print(this_crazyflie_data)

                # Add the data for this body to the ViconData struct
                viconData.crazyflies.append(this_crazyflie_data)

                # Increment the indexing
                current_body_index = current_body_index + 1


            # PUBLISH THE VICON DATA
            self._qualisys_data_publisher.publish(viconData)

            # DEBUGGIN:
            #print(viconData)





    # EXTRACT THE INDEX DICTIONARY FROM 6DOF SETTING XML
    def create_body_index(self, xml_string):
        # Parse the element tree of the XML string
        xml = ET.fromstring(xml_string)

        # Initliase the "body_to_index" return variable
        body_to_index = {}
        # Iterate through the approriate elements of the XML element tree
        for index, body in enumerate(xml.findall("*/Body/Name")):
            # Add the name of this body and its index
            body_to_index[body.text.strip()] = index

        # Return the "body_to_index" variable
        return body_to_index





    # EXTRACT THE INDEX DICTIONARY FROM 6DOF SETTING XML
    def create_list_of_body_names(self, xml_string):
        # Parse the element tree of the XML string
        xml = ET.fromstring(xml_string)

        # Initliase the "list_of_names" return variable
        list_of_names = list()
        # Iterate through the approriate elements of the XML element tree
        for index, body in enumerate(xml.findall("*/Body/Name")):
            # Add the name of this body to the list
            list_of_names.append(body.text.strip())

        # Return the "list_of_names" variable
        return list_of_names





# THIS INITIALISES THE ROS NODE
if __name__ == '__main__':

    # STARTING THE ROS-NODE
    global node_name
    node_name = "ViconDataPublisher"
    rospy.init_node(node_name, anonymous=True)

    # GET THE NAMESPACE OF THIS NODE
    global ros_namespace
    ros_namespace = rospy.get_namespace()
    
    # PUBLISHER FOR THE MOTION CAPTURE DATA
    #global qualisys_data_publisher
    #qualisys_data_publisher = rospy.Publisher(node_name + '/ViconData', ViconData, queue_size=1)

    # IP ADDRESS OF THE HOST "QUALISYS COMPUTER"
    # > This is specified in the "QualisysConfig.yaml" file
    # > That yaml file is added to this node during launch
    # > The "Qualisys Computer" runs the "QTM" software that
    #   is the heart of the Qualisys Motion Capture system
    number_of_fails = 0
    while not rospy.has_param("~qtm_host_ip_address"):
        number_of_fails = number_of_fails + 1 
        if number_of_fails > 9:
            print("[QUALISYS DATA PUBLISHER] ERROR, qtm_host_ip_address parameter not found.")
            number_of_fails = 0;
        time.sleep(0.05)

    #global qtm_ip
    qtm_ip = rospy.get_param("~qtm_host_ip_address")

    print("[QUALISYS DATA PUBLISHER] qtm_host_ip_address parameter = " , qtm_ip )
    
    # INFORM THE USER
    #rospy.loginfo("[QUALISYS DATA PUBLISHER] Now attempting to connection to QTM server at IP address: %s", qtm_ip)
    

    # Initialise a "CrazyRadioClient" type variable that handles
    # all communication over the CrazyRadio
    global qtm_client
    qtm_client = QualisysQtmClient()

    # Connect to the QTM server
    asyncio.get_event_loop().run_until_complete( qtm_client.connect_to_qtm() )



    #asyncio.get_event_loop().run_until_complete( qtm_client.start_streaming() )

    # Start streaming the data
    asyncio.ensure_future( qtm_client.start_streaming() )
    asyncio.get_event_loop().run_forever()
    
    # PAUSE FOR A SECOND AND THEN SPIN
    time.sleep(1.0)
    print("[QUALISYS DATA PUBLISHER] Node READY :-)")
    rospy.spin()

    # DISCONNECT FROM THE QTM SERVER
    #print("[QUALISYS DATA PUBLISHER] Disconnecting from QTM server")
    #qtm_client.stop_streaming()
    
    # Wait for diconnection to complete
    #time.sleep(1.0)
    #print("[QUALISYS DATA PUBLISHER] Disconnected from QTM server")

    # CLOSE THE LOGGING "BAG"
    #bag.close()
    #rospy.loginfo("[QUALISYS DATA PUBLISHER] ROS logging bag is closed")