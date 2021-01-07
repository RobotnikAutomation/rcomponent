#!/usr/bin/env python
from rcomponent.rcomponent import *

# Insert here general imports:
import math

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse


class ?RCNode(RComponent):
    """
    ?brief
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.example_subscriber_name = rospy.get_param(
            '~example_subscriber_name', 'example')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.example_sub = rospy.Subscriber(
            self.example_subscriber_name, String, self.example_sub_cb)

        self.data_pub = rospy.Publisher(
            '~data', String, queue_size=10)
        self.data_stamped_pub = rospy.Publisher(
            '~data_stamped', StringStamped, queue_size=10)

        self.example_server = rospy.Service(
            '~example', Trigger, self.example_server_cb)

        return 0

    def init_state(self):
        self.data = String()

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with data

        data_stamped = StringStamped()
        data_stamped.header.stamp = rospy.Time.now()
        data_stamped.string = self.data

        self.data_pub.publish(self.data)
        self.data_stamped_pub.publish(data_stamped)

        return RComponent.ready_state(self)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def example_sub_cb(self, msg):
        ros.loginfo("Received msg: " + msg.data)

    def example_server_cb(self, req):
        ros.loginfo("Received srv trigger petition.")

        response = TriggerResponse()
        response.success = True
        response.message = "Received srv trigger petition."
        return response