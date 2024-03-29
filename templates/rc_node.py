#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

        super().__init__()

    def ros_read_params(self):
        """Gets params from param server"""
        super().ros_read_params()

        self.example_subscriber_name = rospy.get_param(
            '~example_subscriber_name', 'example')

    def ros_setup(self):
        """Creates and inits ROS components"""

        super().ros_setup()

        # Publisher
        self.status_pub = rospy.Publisher(
            '~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher(
            '~status_stamped', StringStamped, queue_size=10)

        # Subscriber
        self.example_sub = rospy.Subscriber(
            self.example_subscriber_name, String, self.example_sub_cb)
        super().add_topics_health(self.example_sub, topic_id='example_sub', timeout=1.0, required=False)

        # Service
        self.example_server = rospy.Service(
            '~example', Trigger, self.example_server_cb)

        return 0

    def init_state(self):
        self.status = String()
        
        super().setup()
        return super().init_state()

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return super().ready_state()

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        return super().ready_state()

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return super().shutdown()

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return super().switch_to_state(new_state)

    def example_sub_cb(self, msg):
        rospy.logwarn("Received msg: " + msg.data)
        self.tick_topics_health('example_sub')

    def example_server_cb(self, req):
        rospy.logwarn("Received srv trigger petition.")

        response = TriggerResponse()
        response.success = True
        response.message = "Received srv trigger petition."
        return response