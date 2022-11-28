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

class LogClient:
    'Class to interact with a logs server'

    def __init__(self, id, service_ns):
        # id to identify the component
        self._id = id

        # Service client stuff
        self.service_ns = service_ns
        self.client = rospy.ServiceProxy(service_ns, LoggerQuery)
    
    def add_debug(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_DEBUG

        return self.__send_request(query, verbose)
    
    def add_info(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_INFO

        return self.__send_request(query, verbose)
    
    def add_warning(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_WARNING

        return self.__send_request(query, verbose)
    
    def add_error(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_ERROR

        return self.__send_request(query, verbose)
    
    def add_user(self, description, tag, verbose=True):
        query = self.__build_base_query(description, tag)
        query.log_level = Logger.LOG_LEVEL_USER

        return self.__send_request(query, verbose)

    def __build_base_query(self, description, tag):
        query = Logger()
        query.component = self._id
        query.description = description
        query.tag = tag
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
            

