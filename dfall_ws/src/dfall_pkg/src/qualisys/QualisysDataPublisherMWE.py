#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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




import roslib;
import rospy


#import rosbag
from rospkg import RosPack



# Import the library for asynchronous I/O
import asyncio

# Import the Qualisys Track Manager (QTM) Library
import qtm


def on_packet(packet):
    """ Callback function that is called everytime a data packet arrives from QTM """
    print("Framenumber: {}".format(packet.framenumber))
    #header, markers = packet.get_3d_markers()
    #print("Component info: {}".format(header))
    #for marker in markers:
    #    print("\t", marker)


async def setup():
    """ Main function """
    connection = await qtm.connect("10.42.0.15")

    if connection is None:
        print("[QUALISYS DATA PUBLISHER] connection is NONE")
        return

    async with qtm.TakeControl(connection, "password"):
        await connection.stream_frames(frames="frequencydivisor:50",components=["3d"], on_packet=on_packet)

        # Wait asynchronously 5 seconds
        await asyncio.sleep(2)

        # Stop streaming
        await self._qtm_connection.stream_frames_stop()


if __name__ == "__main__":
	# Starting the ROS-node
    global node_name
    node_name = "QualisysDataPublisherMWE"
    rospy.init_node(node_name, anonymous=True)

    # Get the namespace of this node
    global ros_namespace
    ros_namespace = rospy.get_namespace()

    asyncio.ensure_future(setup())
    asyncio.get_event_loop().run_forever()

    # PAUSE FOR A SECOND AND THEN SPIN
    time.sleep(1.0)
    print("[QUALISYS DATA PUBLISHER] Node READY :-)")
    rospy.spin()
