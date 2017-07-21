/*! \class RComponent
 *  \file RComponent.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2014
 *  \brief Class to define a standard and shared structure (attributes & methods) for all the components
 * 			Single thread by default
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <rcomponent/rcomponent.h>
#include <robotnik_msgs/State.h>

/*! \fn RComponent::RComponent()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
RComponent::RComponent(ros::NodeHandle h):nh_(h), pnh_("~"){
	// Set main flags to false
	ros_initialized = initialized = running = false;
	// reads params from server
	rosReadParams();
	
	if(desired_freq_ <= 0.0)
		desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;
		
	state = robotnik_msgs::State::INIT_STATE;
	// Realizar para cada una de las clases derivadas
	component_name.assign("RComponent");
	
}

/*! \fn RComponent::~RComponent()
 * Destructor by default
*/
RComponent::~RComponent(){
	
}

/*! \fn int RComponent::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int RComponent::setup(){
	// Checks if has been initialized
	if(initialized){
		ROS_INFO("%s::Setup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	

	initialized = true;

	return OK;
}

/*! \fn int RComponent::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RComponent::shutdown(){
	
	if(running){
		ROS_INFO("%s::Shutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!initialized){
		ROS_INFO("%s::Shutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////
	
	
	initialized = false;

	return OK;
}


/*! \fn int RComponent::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RComponent::start(){
	// Performs ROS setup
	rosSetup();
	
	if(running){
		ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
		return THREAD_RUNNING;
	}
	
	ROS_INFO("%s started", component_name.c_str());
	
	running = true;
	
	// Executes the control loop
	controlLoop();
	
	return OK;

}

/*! \fn int RComponent::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int RComponent::stop(){
	
	if(!running){
		ROS_INFO("%s::stop: Thread not running", component_name.c_str());
	
		return THREAD_NOT_RUNNING;
	}
	//
	///////////////////////////////////////////////////
	// Stops another subcomponents, if it's necessary //
	///////////////////////////////////////////////////
	//
	ROS_INFO("%s::Stop: Stopping the component", component_name.c_str());
	
	running = false;

	usleep(100000);

	return OK;
}

/*!	\fn void RComponent::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void RComponent::controlLoop(){
	ROS_INFO("%s::controlLoop(): Init", component_name.c_str());
	ros::Rate r(desired_freq_);  
	ros::Time t1,t2;
	while(running && ros::ok()) {
		
		t1 = ros::Time::now();
		
		switch(state){
			
			case robotnik_msgs::State::INIT_STATE:
				initState();
			break;
			
			case robotnik_msgs::State::STANDBY_STATE:
				standbyState();
			break;
			
			case robotnik_msgs::State::READY_STATE:
				readyState();
			break;
			
			case robotnik_msgs::State::SHUTDOWN_STATE:
				shutdownState();
			break;
			
			case robotnik_msgs::State::EMERGENCY_STATE:
				emergencyState();
			break;
			
			case robotnik_msgs::State::FAILURE_STATE:
				failureState();
			break;
		
		}
		
		allState();
		
		ros::spinOnce();
		r.sleep();
		
		t2 = ros::Time::now();
		
		real_freq = 1.0/(t2 - t1).toSec();
		
	}
	
	shutdownState();
	// Performs ROS Shutdown
	rosShutdown();

	ROS_INFO("%s::controlLoop(): End", component_name.c_str());

}

/*!	\fn void RComponent::initState()
 *	\brief Actions performed on initial 
 * 	Setups the component
*/
void RComponent::initState(){
	// If component setup is successful goes to STANDBY (or READY) state
	if(setup() != ERROR){
		switchToState(robotnik_msgs::State::STANDBY_STATE);
	}
}

/*!	\fn void RComponent::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void RComponent::shutdownState(){
	
	if(shutdown() == OK){
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
}

/*!	\fn void RComponent::standbyState()
 *	\brief Actions performed on Standby state
*/
void RComponent::standbyState(){

}

/*!	\fn void RComponent::readyState()
 *	\brief Actions performed on ready state
*/
void RComponent::readyState(){

}

/*!	\fn void RComponent::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void RComponent::emergencyState(){

}

/*!	\fn void RComponent::FailureState()
 *	\brief Actions performed on failure state
*/
void RComponent::failureState(){

}

/*!	\fn void RComponent::AllState()
 *	\brief Actions performed on all states
*/
void RComponent::allState(){
	rosPublish();
}

/*!	\fn double RComponent::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double RComponent::getUpdateRate(){
	return desired_freq_;
}

/*!	\fn int RComponent::getState()
 * 	\brief returns the state of the component
*/
int RComponent::getState(){
	return state;
}

/*!	\fn char *RComponent::getStateString()
 *	\brief Gets the state of the component as string
*/
char *RComponent::getStateString(){
	return getStateString(state);
}

/*!	\fn char *RComponent::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *RComponent::getStateString(int state){
	switch(state){
		case robotnik_msgs::State::INIT_STATE:
			return (char *)"INIT";
		break;
		case robotnik_msgs::State::STANDBY_STATE:
			return (char *)"STANDBY";
		break;
		case robotnik_msgs::State::READY_STATE:
			return (char *)"READY";
		break;
		case robotnik_msgs::State::EMERGENCY_STATE:
			return (char *)"EMERGENCY";
		break;
		case robotnik_msgs::State::FAILURE_STATE:
			return (char *)"FAILURE";
		break;
		case robotnik_msgs::State::SHUTDOWN_STATE:
			return (char *)"SHUTDOWN";
		break;
		default:
			return (char *)"UNKNOWN";
		break;
	}
}


/*!	\fn void RComponent::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void RComponent::switchToState(int new_state){
	
	if(new_state == state)
		return;

	// saves the previous state
	previous_state = state;
	ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));	
	state = new_state;

}

/*!	\fn void RComponent::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int RComponent::rosSetup(){
	
	// Checks if has been initialized
	if(ros_initialized){
		ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	state_publisher = pnh_.advertise<robotnik_msgs::State>("state", 1);
	//state_publisher = pnh_.advertise<std_msgs::Empty>("state", 1);
	

	ros_initialized = true;

	return OK;
	
	/*
	
	status_pub_ = pnh_.advertise<agvs_controller::DspicStatus>("status", 1);
	odom_pub_ = pnh_.advertise<nav_msgs::Odometry>(odom_frame_id_, 1);
	calibrate_srv_ = pnh_.advertiseService("calibrate",  &dspic_controller_node::CalibrateSrv, this);
	set_odom_service_ = pnh_.advertiseService("set_odometry", &dspic_controller_node::SetOdometry, this);*/
}


/*!	\fn void RComponent::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void RComponent::rosReadParams(){
	
	pnh_.param("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ);
	
	/* Example 
	pnh_.param<std::string>("port", port_, DEFAULT_DSPIC_PORT);
	pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
	pnh_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
	pnh_.param("publish_tf", publish_tf_, false);
	pnh_.param("desired_freq", desired_freq_, desired_freq_);*/
}

/*!	\fn int RComponent::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int RComponent::rosShutdown(){
	if(running){
		ROS_INFO("%s::rosShutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!ros_initialized){
		ROS_INFO("%s::rosShutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	
	
	ros_initialized = false;

	return OK;
}

/*!	\fn void RComponent::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void RComponent::rosPublish(){
	robotnik_msgs::State msg;
	
	// STATE
	msg.state = this->state;
	msg.desired_freq = this->desired_freq_;
	msg.real_freq = this->real_freq;
	msg.state_description = getStateString();
	
	
	state_publisher.publish(msg);
	
}
