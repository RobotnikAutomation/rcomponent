#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Robotnik Automation SLL
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
from rospy.service import ServiceException

from robotnik_msgs.msg import Logger
from robotnik_msgs.srv import LoggerQuery, LoggerQueryRequest

from datetime import datetime
import threading
import inspect
import os


LOGLEVEL_COLOR_MAPPING = {'DEBUG': '\033[92m',
                          'INFO': '\033[32;20m',
                          'WARNING': '\033[33;20m',
                          'ERROR': '\033[31;20m',
                          'USER': '\033[38;20m',
                          'reset': '\033[0m'}


class LogClient:
    """
    Class to interact with a logs server
    """
    def __init__(self, component):

        super().__init__()
        # Service client stuff
        self.service_ns = 'ddbb_client/logger/insert'
        self.client = rospy.ServiceProxy(service_ns, LoggerQuery)
        self.robot_id = os.environ['HOSTNAME']
        self.component = component
        # Create a dict to store the history of logs
        # It is only used for throttling and logging once, it is not a file storing logs
        self.log_history = dict()
        self.throttle_timer = None
        self.throttle_identical_timer = None
    
    # Normal Log Messages
    
    def logdebug(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_DEBUG
        self.__stdout_log(query)
        self.__send_request(query, verbose)
    
    def loginfo(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_INFO
        self.__stdout_log(query)
        self.__send_request(query, verbose)
    
    def logwarning(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_WARNING
        self.__stdout_log(query)
        self.__send_request(query, verbose)
    
    def logerror(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_ERROR
        self.__stdout_log(query)
        self.__send_request(query, verbose)
    
    def loguser(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_USER
        self.__stdout_log(query)
        self.__send_request(query, verbose)

    # Throttle Log Messages
    
    def logdebug_throttle(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_behavior(self.logdebug, time_period, description, tag, verbose)
    
    def loginfo_throttle(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_behavior(self.loginfo, time_period, description, tag, verbose)

    def logwarning_throttle(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_behavior(self.logwarning, time_period, description, tag, verbose)

    def logerror_throttle(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_behavior(self.logerror, time_period, description, tag, verbose)

    def loguser_throttle(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_behavior(self.loguser, time_period, description, tag, verbose)

    # Throttle Identical Log Messages
    
    def logdebug_throttle_identical(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_identical_behavior(self.logdebug, time_period, description, tag, verbose)
    
    def loginfo_throttle_identical(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_identical_behavior(self.loginfo, time_period, description, tag, verbose)

    def logwarning_throttle_identical(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_identical_behavior(self.logwarning, time_period, description, tag, verbose)

    def logerror_throttle_identical(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_identical_behavior(self.logerror, time_period, description, tag, verbose)

    def loguser_throttle_identical(self, time_period, description, tag, verbose=True):
        self.__perform_throttle_identical_behavior(self.loguser, time_period, description, tag, verbose)

    # Log Once Messages

    def logdebug_once(self, description, tag, verbose=True):
        self.__perform_once_behavior(self.logdebug, description, tag, verbose)
    
    def loginfo_once(self, description, tag, verbose=True):
        self.__perform_once_behavior(self.loginfo, description, tag, verbose)

    def logwarning_once(self, description, tag, verbose=True):
        self.__perform_once_behavior(self.logwarning, description, tag, verbose)

    def logerror_once(self, description, tag, verbose=True):
        self.__perform_once_behavior(self.logerror, description, tag, verbose)

    def loguser_once(self, description, tag, verbose=True):
        self.__perform_once_behavior(self.loguser, description, tag, verbose)

    # Logging Behaviors
    
    def __perform_throttle_behavior(self, log_level_callable, time_period, description, tag, verbose):
        # Get the caller's filename and line number
        curframe = inspect.currentframe()
        callframe = inspect.getouterframes(curframe, 2)[1]
        filename, lineno = callframe.filename, callframe.lineno

        # If the log already exists in the history, throttling is already activated
        if (filename, lineno) in self.log_history.keys():
            return

        else:
            # Store log data in history
            self.log_history[(filename, lineno)] = description
            # Log for the first time the msg
            log_level_callable(description, tag, verbose)
            # Create and start the throttling Timer
            self.throttle_timer = threading.Timer(time_period, self.__erase_log_record, [filename, lineno])
            self.throttle_timer.start()

    def __perform_throttle_identical_behavior(self, log_level_callable, time_period, description, tag, verbose):
        # Get the caller's filename and line number
        curframe = inspect.currentframe()
        callframe = inspect.getouterframes(curframe, 2)[1]
        filename, lineno = callframe.filename, callframe.lineno

        # If the log already exists in the history, throttling is already activated
        if (filename, lineno) in self.log_history.keys() and self.log_history[(filename, lineno)] == description:
            return

        else:
            # Store log data in history
            self.log_history[(filename, lineno)] = description
            # Log for the first time the msg
            log_level_callable(description, tag, verbose)
            # Create and start the throttling Timer, after erasing the previous Timer
            if self.throttle_identical_timer:
                self.throttle_identical_timer.cancel()
            self.throttle_identical_timer = threading.Timer(time_period, self.__erase_log_record, [filename, lineno])
            self.throttle_identical_timer.start()

    def __perform_once_behavior(self, log_level_callable, description, tag, verbose):
        # Get the caller's filename and line number
        curframe = inspect.currentframe()
        callframe = inspect.getouterframes(curframe, 2)[1]
        filename, lineno = callframe.filename, callframe.lineno

        # If the log already exists in the history, throttling is already activated
        if (filename, lineno) in self.log_history.keys():
            return

        else:
            # Store log data in history
            self.log_history[(filename, lineno)] = description
            # Log for the first time the msg
            log_level_callable(description, tag, verbose)

    def __erase_log_record(self, filename, lineno):
        self.log_history.pop((filename, lineno))

    def __stdout_log(self, query):
        log_msg = f'[{query.log_level}] [{query.date_time}] [{query.robot_id}] [{query.component}] [{query.tag}] {query.description}'
        color_code = LOGLEVEL_COLOR_MAPPING[query.log_level]
        reset_color = LOGLEVEL_COLOR_MAPPING['reset']
        print(color_code + log_msg + reset_color)
    
    def __build_base_query(self, description, tag):
        query = Logger()
        query.robot_id = self.robot_id
        query.date_time = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')
        query.component = self.component
        query.tag = tag
        query.description = description
        return query

    def __send_request(self, query, verbose):
        request = LoggerQueryRequest()
        request.query = query
        try:
            self.client.call(request)
        except ServiceException as error:
            rospy.logerr_throttle(2, "%s::LogClient::__send_request: Error sending last log '%s'. %s" \
                %(self._id, query.description, error))
            return False
        
        if verbose == True:
            rospy.loginfo("%s::LogClient::__send_request: Log '%s' correctly added" \
                % (self._id, query.description))
        return True

