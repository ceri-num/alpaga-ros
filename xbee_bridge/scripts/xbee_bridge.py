#!/usr/bin/env python

# Proof of Concept code v0
# XBeeBridge code should be shared / abstracted

import argparse
import sys
import time

from digi.xbee.devices import XBeeDevice
# from xbee_bridge import XBeeBridge
import rospy
from std_msgs.msg import String

# GLOBALS
NODE_ID = "xbee_bridge"

def log(s):
    global NODE_ID
    #print(s)
    rospy.loginfo('['+NODE_ID+'] '+s)

class XBeeBridge:

    def __init__(self, _port, _baud_rate, _node_id, _remote_id):
        self.node_id = _node_id
        self.remote_node_id = _remote_id
        self.port=_port

        self.device = XBeeDevice(_port, _baud_rate)
        self.initDevice()
        # self.connectToRemoteDevice()

        self.initReceivedTopics()

    def initDevice(self):
        retry = 3
        # do while
        while True:
            self.log('Open Xbee device on ' +  self.port)
            try:
                self.device.open()
                time.sleep(0.5)
            except:
                pass

            if self.device.is_open() or retry<0:
                break
            else:
                self.log('Retrying ... ')
                retry = retry-1

        if not self.device.is_open():
            self.log('Xbee Device could not be opened.')
            raise Exception()

        self.log(NODE_ID+' Device initialized')

        self.device.flush_queues()

    def connectToRemoteDevice(self):
        self.xnetwork = self.device.get_network()
        self.remote_device = self.xnetwork.discover_device(self.remote_node_id)
        time.sleep(0.5)
        if self.remote_device is None:
            self.log('Could not find the remote device.')
            self.device.close()
            raise Exception()

    def initReceivedTopics(self):
        rospy.Subscriber(self.node_id+"/send/"+self.remote_node_id, String, self.send_through_xbee_callback)

    def send_through_xbee_callback(self, _data):
        self.log("Send data to " + self.remote_node_id)
        # self.device.send_data_async(self.remote_device, str(_data.data))
        self.device.send_data_broadcast(str(_data.data))

    def terminate(self):
        self.log('Terminating Session...')
        if self.device is not None and self.device.is_open():
            self.device.close()

    def log(self,s):
        # print(s)
        rospy.loginfo('['+self.node_id+'] '+s)

def main():
    global NODE_ID
    rospy.init_node(NODE_ID, anonymous=True)
    NODE_ID = rospy.get_name()
    NODE_ID = NODE_ID[1:]

    # Local Xbee module
    PORT = rospy.get_param('/'+NODE_ID+'/xbee_port', '/dev/ttyUSB0')
    BAUD_RATE = rospy.get_param('/'+NODE_ID+'/xbee_baud', 9600)

    # Remote Xbee module
    REMOTE_NODE_ID=rospy.get_param('/'+NODE_ID+'/remote_node_id', 'xbee_remote')

    log(" +-------------------------------------------------+")
    log(" NODE_ID = "+NODE_ID)
    log(" REMOTE_NODE_ID = "+REMOTE_NODE_ID)
    log(" PORT = "+PORT)
    log(" BAUD_RATE = "+str(BAUD_RATE))
    log(" +-------------------------------------------------+")

    ros_rate = rospy.Rate(100)

    try:
        bridge = XBeeBridge(PORT, BAUD_RATE, NODE_ID, REMOTE_NODE_ID)
    except:
        sys.exit(1)

    # ROS Publisher
    publisher_receivedData = rospy.Publisher(NODE_ID+"/receive/"+REMOTE_NODE_ID, String, queue_size=10)

    log('Awaiting conversation...\n')

    try:
        while not rospy.is_shutdown():
            xbee_message = bridge.device.read_data()

            if xbee_message is not None:
                log('Message received\n')
                message = xbee_message.data.decode()

                # if message == 's':
                # ...

                publisher_receivedData.publish(message)

            ros_rate.sleep()
    finally:
        bridge.terminate()

if __name__ == "__main__":
    #try:
    main()
    #except rospy.ROSInterruptException:
    #    pass
