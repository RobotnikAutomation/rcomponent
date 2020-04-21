#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

import time
import threading

from robotnik_msgs.msg import State

DEFAULT_FREQ = 10.0

# Class Template of Robotnik component for Pyhton


class RComponent:

    def __init__(self):
        self._node_name = rospy.get_name()
        self.ros_read_params()

        self._real_freq = 0.0
        # Saves the state of the component
        self._state = State.INIT_STATE
        # Saves the previous state
        self._previous_state = State.INIT_STATE
        # flag to control the initialization of the component
        self._initialized = False
        # flag to control the initialization of ROS stuff
        self._ros_initialized = False
        # flag to control that the control loop is running
        self._running = False
        # Variable used to control the loop frequency
        self._time_sleep = 1.0 / self._desired_freq
        # State msg to publish
        self._msg_state = State()
        # Timer to publish state
        self._publish_state_timer = 1

        self._t_publish_state = threading.Timer(self._publish_state_timer, self.publish_ros_state)

    def ros_read_params(self):
        '''
            Gets params from param server
        '''

        try:
            self._desired_freq = rospy.get_param('~desired_freq', default=DEFAULT_FREQ)
        except rospy.ROSException, e:
            rospy.logerr('%s' % (e))
            exit(-1)

        # Checks value of freq
        if self._desired_freq <= 0.0:
            rospy.loginfo('%s::init: Desired freq to %f is not possible. Setting _desired_freq to %f' %
                          (self._node_name, self._desired_freq, DEFAULT_FREQ))
            self._desired_freq = DEFAULT_FREQ

    def setup(self):
        '''
                Initializes de hand
                @return: True if OK, False otherwise
        '''
        self._initialized = True

        return 0

    def ros_setup(self):
        '''
                Creates and inits ROS components
        '''
        if self._ros_initialized:
            return 0

        # Publishers
        self._state_publisher = rospy.Publisher('~state', State, queue_size=10)
        # Subscribers
        # topic_name, msg type, callback, queue_size
        # self.topic_sub = rospy.Subscriber('topic_name', Int32, self.topic_cb, queue_size = 10)
        # Service Servers
        # self.service_server = rospy.Service('~service', Empty, self.service_cb)
        # Service Clients
        # self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
        # ret = self.service_client.call(ServiceMsg)

        self._ros_initialized = True

        self.publish_ros_state()

        return 0

    def shutdown(self):
        '''
                Shutdowns device
                @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self._running or not self._initialized:
            return -1
        rospy.loginfo('%s::shutdown' % self._node_name)

        # Cancels current timers
        self._t_publish_state.cancel()

        self._state_publisher.unregister()

        self._initialized = False

        return 0

    def ros_shutdown(self):
        '''
                Shutdows all ROS components
                @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self._running or not self._ros_initialized:
            return -1

        # Performs ROS topics & services shutdown
        self._state_publisher.unregister()

        self._ros_initialized = False

        return 0

    def stop(self):
        '''
                Creates and inits ROS components
        '''
        self._running = False

        return 0

    def start(self):
        '''
                Runs ROS configuration and the main control loop
                @return: 0 if OK
        '''
        self.ros_setup()

        if self._running:
            return 0

        self._running = True

        self.control_loop()

        return 0

    def control_loop(self):
        '''
                Main loop of the component
                Manages actions by state
        '''

        while self._running and not rospy.is_shutdown():
            t1 = time.time()

            if self._state == State.INIT_STATE:
                self.init_state()

            elif self._state == State.STANDBY_STATE:
                self.standby_state()

            elif self._state == State.READY_STATE:
                self.ready_state()

            elif self._state == State.EMERGENCY_STATE:
                self.emergency_state()

            elif self._state == State.FAILURE_STATE:
                self.failure_state()

            elif self._state == State.SHUTDOWN_STATE:
                self.shutdown_state()

            self.all_state()

            t2 = time.time()
            tdiff = (t2 - t1)

            t_sleep = self._time_sleep - tdiff

            if t_sleep > 0.0:
                try:
                    rospy.sleep(t_sleep)
                except rospy.exceptions.ROSInterruptException:
                    rospy.loginfo('%s::control_loop: ROS interrupt exception' % self._node_name)
                    self._running = False

            t3 = time.time()
            self._real_freq = 1.0/(t3 - t1)

        self._running = False
        # Performs component shutdown
        self.shutdown_state()
        # Performs ROS shutdown
        self.ros_shutdown()
        rospy.loginfo('%s::control_loop: exit control loop' % self._node_name)

        return 0

    def ros_publish(self):
        '''
                Publish topics at standard frequency
        '''

        return 0

    def init_state(self):
        '''
                Actions performed in init state
        '''

        if not self._initialized:
            self.setup()

        else:
            self.switch_to_state(State.STANDBY_STATE)

        return

    def standby_state(self):
        '''
                Actions performed in standby state
        '''
        self.switch_to_state(State.READY_STATE)

        return

    def ready_state(self):
        '''
                Actions performed in ready state
        '''

        return

    def shutdown_state(self):
        '''
                Actions performed in shutdown state
        '''
        if self.shutdown() == 0:
            self.switch_to_state(State.INIT_STATE)

        return

    def emergency_state(self):
        '''
                Actions performed in emergency state
        '''

        return

    def failure_state(self):
        '''
                Actions performed in failure state
        '''

        return

    def switch_to_state(self, new_state):
        '''
                Performs the change of state
        '''
        if self._state != new_state:
            self._previous_state = self._state
            self._state = new_state
            rospy.loginfo('%s::switch_to_state: from %s to %s' % (self._node_name,
                                                                  self.state_to_string(self._previous_state), self.state_to_string(self._state)))

        return

    def all_state(self):
        '''
                Actions performed in all states
        '''
        self.ros_publish()

        return

    def state_to_string(self, state):
        '''
                @param state: state to set
                @type state: State
                @returns the equivalent string of the state
        '''
        if state == State.INIT_STATE:
            return 'INIT_STATE'

        elif state == State.STANDBY_STATE:
            return 'STANDBY_STATE'

        elif state == State.READY_STATE:
            return 'READY_STATE'

        elif state == State.EMERGENCY_STATE:
            return 'EMERGENCY_STATE'

        elif state == State.FAILURE_STATE:
            return 'FAILURE_STATE'

        elif state == State.SHUTDOWN_STATE:
            return 'SHUTDOWN_STATE'
        else:
            return 'UNKNOWN_STATE'

    def publish_ros_state(self):
        '''
                Publish the State of the component at the desired frequency
        '''
        self._msg_state.state = self._state
        self._msg_state.state_description = self.state_to_string(self._state)
        self._msg_state.desired_freq = self._desired_freq
        self._msg_state.real_freq = self._real_freq

        if rospy.is_shutdown() == False:
            self._state_publisher.publish(self._msg_state)
            self._t_publish_state = threading.Timer(self._publish_state_timer, self.publish_ros_state)
            self._t_publish_state.start()

    """
    def topic_cb(self, msg):
    	'''
    		Callback for inelfe_video_manager state
    		@param msg: received message
    		@type msg: std_msgs/Int32
    	'''
    	# DEMO
    	rospy.loginfo('RComponent:topic_cb')


    def service_cb(self, req):
    	'''
    		ROS service server
    		@param req: Required action
    		@type req: std_srv/Empty
    	'''
    	# DEMO
    	rospy.loginfo('RComponent:service_cb')
    """


'''
def main():

	rospy.init_node("rcomponent")


	_name = rospy.get_name().replace('/','')

	arg_defaults = {
	  'topic_state': 'state',
	  '_desired_freq': DEFAULT_FREQ,
	}

	args = {}

	for name in arg_defaults:
		try:
			if rospy.search_param(name):
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))


	rc_node = RComponent(args)

	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
'''
