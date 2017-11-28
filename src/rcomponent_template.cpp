/*! \class RComponent
 *  \file rcomponent_template.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2015
 *  \brief Class to define a standard and shared structure (attributes & methods) for all the components
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

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <robotnik_msgs/State.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <rcomponent/rcomponent_log_macros.h>
//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ 40.0

using namespace std;

//! Defines return values for methods and functions
enum ReturnValue
{
  OK = 0,
  INITIALIZED,
  THREAD_RUNNING,
  ERROR = -1,
  NOT_INITIALIZED = -2,
  THREAD_NOT_RUNNING = -3,
  COM_ERROR = -4,
  NOT_ERROR = -5
};

//! Class Rcomponent
class RComponent
{
protected:
  //! Controls if has been initialized succesfully
  bool initialized, ros_initialized;
  //! Controls the execution of the RComponent's thread
  bool running;

  //! State of the RComponent
  int state;
  //! State before
  int previous_state;
  //!	Saves the name of the component
  string component_name;
  //! ROS node handle
  ros::NodeHandle nh_;
  //! Private ROS node handle
  ros::NodeHandle pnh_;
  //! Desired loop frequency
  double desired_freq_, real_freq;

  //! Publish the component state
  ros::Publisher state_publisher_;
  // Examples:
  // ros::Subscriber sub_; // topic subscriber
  // ros::ServiceServer service_server_; // service server
  // ros::ServiceClient service_client_; // service client

  //! General status diagnostic updater
  diagnostic_updater::Updater* diagnostic_;

public:
  //! Public constructor
  RComponent(ros::NodeHandle h);
  //! Public destructor
  ~RComponent();

  //! Starts the control loop of the component and its subcomponents
  //! @return OK
  //! @return ERROR starting the thread
  //! @return RUNNING if it's already running
  //! @return NOT_INITIALIZED if it's not initialized
  virtual int start();
  //! Stops the main control loop of the component and its subcomponents
  //! @return OK
  //! @return ERROR if any error has been produced
  //! @return NOT_RUNNING if the main thread isn't running
  virtual int stop();
  //! Returns the general state of the RComponent
  int getState();
  //! Returns the general state of the RComponent as string
  char* getStateString();
  //! Returns the general state as string
  char* getStateString(int state);
  //! Method to get current update rate of the thread
  //! @return pthread_hz
  double getUpdateRate();

protected:
  //! Configures and initializes the component
  //! @return OK
  //! @return INITIALIZED if the component is already intialized
  //! @return ERROR
  int setup();
  //! Closes and frees the reserved resources
  //! @return OK
  //! @return ERROR if fails when closes the devices
  //! @return RUNNING if the component is running
  //! @return NOT_INITIALIZED if the component is not initialized
  int shutdown();
  //! All core component functionality is contained in this thread.
  //!	All of the RComponent component state machine code can be found here.
  void controlLoop();
  //! Actions performed on initial state
  void initState();
  //! Actions performed on standby state
  void standbyState();
  //! Actions performed on ready state
  void readyState();
  //! Actions performed on the emergency state
  void emergencyState();
  //! Actions performed on Failure state
  void failureState();
  //! Actions performed on Shudown state
  void shutdownState();
  //! Actions performed in all states
  void allState();
  //! Switches between states
  void switchToState(int new_state);
  //! Setups all the ROS' stuff
  int rosSetup();
  //! Shutdowns all the ROS' stuff
  int rosShutdown();
  //! Reads data a publish several info into different topics
  void rosPublish();
  //! Reads params from params server
  void rosReadParams();

  // Examples
  // void topicCallback(const std_msgs::StringConstPtr& message); // Callback for a subscriptor
  // bool serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // Callback for a
  // service server

  //! Diagnostic updater callback
  void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat);
};

/*! \fn RComponent::RComponent()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
RComponent::RComponent(ros::NodeHandle h) : nh_(h), pnh_("~")
{
  // Set main flags to false
  ros_initialized = initialized = running = false;
  // reads params from server
  rosReadParams();

  if (desired_freq_ <= 0.0)
    desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;

  state = robotnik_msgs::State::INIT_STATE;
  // Realizar para cada una de las clases derivadas
  component_name.assign("RComponent");
}

/*! \fn RComponent::~RComponent()
 * Destructor by default
*/
RComponent::~RComponent()
{
}

/*! \fn int RComponent::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int RComponent::setup()
{
  // Checks if has been initialized
  if (initialized)
  {
    RCOMPONENT_INFO("Already initialized");

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
int RComponent::shutdown()
{
  if (running)
  {
    RCOMPONENT_INFO("Impossible while thread running, first must be stopped");
    return THREAD_RUNNING;
  }
  if (!initialized)
  {
    RCOMPONENT_INFO("Impossible because of it's not initialized");
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
int RComponent::start()
{
  // Performs ROS setup
  rosSetup();

  if (running)
  {
    RCOMPONENT_INFO("the component's thread is already running");
    return THREAD_RUNNING;
  }

  RCOMPONENT_INFO("Started");

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
int RComponent::stop()
{
  if (!running)
  {
    RCOMPONENT_INFO("Thread not running");

    return THREAD_NOT_RUNNING;
  }
  //
  ///////////////////////////////////////////////////
  // Stops another subcomponents, if it's necessary //
  ///////////////////////////////////////////////////
  //
  RCOMPONENT_INFO("Stopping the component");

  running = false;

  usleep(100000);

  return OK;
}

/*!	\fn void RComponent::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void RComponent::controlLoop()
{
  RCOMPONENT_INFO("Init");
  ros::Rate r(desired_freq_);
  ros::Time t1, t2;
  while (running && ros::ok())
  {
    t1 = ros::Time::now();

    switch (state)
    {
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

    real_freq = 1.0 / (t2 - t1).toSec();
  }

  shutdownState();
  // Performs ROS Shutdown
  rosShutdown();

  RCOMPONENT_INFO("End");
}

/*!	\fn void RComponent::initState()
 *	\brief Actions performed on initial
 * 	Setups the component
*/
void RComponent::initState()
{
  // If component setup is successful goes to STANDBY (or READY) state
  if (setup() != ERROR)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

/*!	\fn void RComponent::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void RComponent::shutdownState()
{
  if (shutdown() == OK)
  {
    switchToState(robotnik_msgs::State::INIT_STATE);
  }
}

/*!	\fn void RComponent::standbyState()
 *	\brief Actions performed on Standby state
*/
void RComponent::standbyState()
{
}

/*!	\fn void RComponent::readyState()
 *	\brief Actions performed on ready state
*/
void RComponent::readyState()
{
}

/*!	\fn void RComponent::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void RComponent::emergencyState()
{
}

/*!	\fn void RComponent::FailureState()
 *	\brief Actions performed on failure state
*/
void RComponent::failureState()
{
}

/*!	\fn void RComponent::AllState()
 *	\brief Actions performed on all states
*/
void RComponent::allState()
{
  diagnostic_->update();

  rosPublish();
}

/*!	\fn double RComponent::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double RComponent::getUpdateRate()
{
  return desired_freq_;
}

/*!	\fn int RComponent::getState()
 * 	\brief returns the state of the component
*/
int RComponent::getState()
{
  return state;
}

/*!	\fn char *RComponent::getStateString()
 *	\brief Gets the state of the component as string
*/
char* RComponent::getStateString()
{
  return getStateString(state);
}

/*!	\fn char *RComponent::getStateString(int state)
 *	\brief Gets the state as a string
*/
char* RComponent::getStateString(int state)
{
  switch (state)
  {
    case robotnik_msgs::State::INIT_STATE:
      return (char*)"INIT";
      break;
    case robotnik_msgs::State::STANDBY_STATE:
      return (char*)"STANDBY";
      break;
    case robotnik_msgs::State::READY_STATE:
      return (char*)"READY";
      break;
    case robotnik_msgs::State::EMERGENCY_STATE:
      return (char*)"EMERGENCY";
      break;
    case robotnik_msgs::State::FAILURE_STATE:
      return (char*)"FAILURE";
      break;
    case robotnik_msgs::State::SHUTDOWN_STATE:
      return (char*)"SHUTDOWN";
      break;
    default:
      return (char*)"UNKNOWN";
      break;
  }
}

/*!	\fn void RComponent::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void RComponent::switchToState(int new_state)
{
  if (new_state == state)
    return;

  // saves the previous state
  previous_state = state;
  RCOMPONENT_INFO("%s -> %s", getStateString(state), getStateString(new_state));
  state = new_state;
}

/*!	\fn void RComponent::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int RComponent::rosSetup()
{
  // Checks if has been initialized
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return INITIALIZED;
  }

  // Publishers
  state_publisher_ = pnh_.advertise<robotnik_msgs::State>("state", 1);

  ros_initialized = true;

  /*
  // EXAMPLES
  // Subscribers
  // topic, queue, callback
  sub_ = nh_.subscribe("topic name", 10,  &RComponent::topicCallback, this);

  // Services server
  service_server_ = pnh_.advertiseService("service name", &RComponent::serviceServerCb, this);
  // Services client
  service_client_ = nh_.serviceClient<std_srvs::Empty>("service client name");

  */

  // Sets up the diagnostic updater
  diagnostic_ = new diagnostic_updater::Updater();

  diagnostic_->setHardwareID("RComponent");
  diagnostic_->add("State", this, &RComponent::diagnosticUpdate);
  diagnostic_->broadcast(0, "Doing important initialization stuff.");
  return OK;
}

/*!	\fn void RComponent::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void RComponent::rosReadParams()
{
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
int RComponent::rosShutdown()
{
  if (running)
  {
    RCOMPONENT_INFO("Impossible while thread running, first must be stopped");
    return THREAD_RUNNING;
  }
  if (!ros_initialized)
  {
    RCOMPONENT_INFO("Impossible because of it's not initialized");
    return NOT_INITIALIZED;
  }

  ros_initialized = false;

  return OK;
}

/*!	\fn void RComponent::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void RComponent::rosPublish()
{
  robotnik_msgs::State msg;

  // STATE
  msg.state = this->state;
  msg.desired_freq = this->desired_freq_;
  msg.real_freq = this->real_freq;
  msg.state_description = getStateString();

  state_publisher_.publish(msg);
}

/*!	\fn void RComponent::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
 * 	\brief Callback to update the component diagnostic
*/
void RComponent::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE ||
      state == robotnik_msgs::State::STANDBY_STATE)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
  else if (state == robotnik_msgs::State::EMERGENCY_STATE)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");

  stat.add("State", getStateString());
}

// EXAMPLE: Callback handler associated with the subscriber
/*void RComponent::topicCallback(const std_msgs::StringConstPtr& message)
{
  RCOMPONENT_INFO("callback: Received msg: %s", message->data.c_str());
}*/

// Callback handler for the service server
/*bool RComponent::serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  RCOMPONENT_INFO("serviceServerCb: Received server");
  // Calls the client service
  std_srvs::Empty service;

  if(service_client_.call(service)){
    RCOMPONENT_INFO("serviceServerCb: calling service");
  }else{
    RCOMPONENT_ERROR("serviceServerCb: Error connecting service %s", service_client_name_.c_str());
  }

  return true;
}*/

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcomponent");

  ros::NodeHandle n;
  RComponent controller(n);

  controller.start();

  return (0);
}
