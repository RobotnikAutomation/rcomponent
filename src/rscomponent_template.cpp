/*! \class RSComponent
 *  \file rscomponent_template.cpp
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

#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <rcomponent/rcomponent_log_macros.h>

#define RSCOMPONENT_MIN_COMMAND_REC_FREQ 1.0
#define RSCOMPONENT_MAX_COMMAND_REC_FREQ 10.0
#define RSCOMPONENT_DEFAULT_FREQ 20.0

class RSComponent
{
public:
  self_test::TestRunner self_test_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;
  diagnostic_updater::Updater diagnostic_;                           // General status diagnostic updater
  diagnostic_updater::FrequencyStatus freq_diag_;                    // Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic* subs_command_freq;  // Topic reception frequency diagnostics
  ros::Time last_command_time_;  // Last moment when the component received a command
  ros::Time last_read_time_;     // Last moment when the positions were read
  diagnostic_updater::FunctionDiagnosticTask command_freq_;
  std::string component_name;

  //! Node running
  bool running;

  /* EXAMPLES */
  // Ros service stop
  // ros::ServiceServer srv_stop_;
  // Publishers
  // ros::Publisher joint_state_pub_;
  // Subscribers
  // ros::Subscriber joint_state_sub_;

  //! Flag to inform about shutdown
  bool shutting_down_;

  bool bReceiving_;

  //! Read write cycle measured frequency
  double cycle_freq_;

  /*!	\fn RSComponent::RSComponent()
   * 	\brief Public constructor
  */
  RSComponent(ros::NodeHandle h)
    : self_test_()
    , diagnostic_()
    , node_handle_(h)
    , private_node_handle_("~")
    , desired_freq_(RSCOMPONENT_DEFAULT_FREQ)
    , freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
    , command_freq_("Command frequency check", boost::bind(&RSComponent::checkCommandSubscriber, this, _1))
    , component_name("Component")
  {
    running = false;
    ros::NodeHandle RSComponent(node_handle_, "RSComponent");

    // READ ROS PARAMS
    // private_node_handle_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_node_handle_.param<double>("desired_freq", desired_freq_, RSCOMPONENT_DEFAULT_FREQ);

    // Self test
    self_test_.add("Connect Test", this, &RSComponent::connectTest);

    // Component Setup
    // Opens/setups devices

    /* EXAMPLES */
    // Services and Topics
    // Service to stop robot
    // srv_stop_ = private_node_handle_.advertiseService("stop", &RSComponent::srvCallbackStop, this);
    // Publish joint states for wheel motion visualization
    // joint_state_pub_ = private_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // Subscribing
    // Start listening to command messages. There is a queue size of 1k so that
    // we don't accidentally miss commands that are sent to us in batches for many joints at once.
    // joint_state_sub_ = private_node_handle_.subscribe<sensor_msgs::JointState>("/joint_commands", 1000,
    // &RSComponent::cmdJointStateCallback, this);

    // Component frequency diagnostics
    diagnostic_.setHardwareID("rscomponent");
    diagnostic_.add("RSComponent Diagnostic", this, &RSComponent::controllerDiagnostic);
    diagnostic_.add(freq_diag_);
    diagnostic_.add(command_freq_);

    // Topics freq control for command topics
    double min_freq = RSCOMPONENT_MIN_COMMAND_REC_FREQ;  // If you update these values, the
    double max_freq = RSCOMPONENT_MAX_COMMAND_REC_FREQ;  // HeaderlessTopicDiagnostic will use the new values.
    RCOMPONENT_INFO("Desired freq %5.2f", desired_freq_);
    // Sets the topic frequency we want to monitorize
    subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(
        "/joint_commands", diagnostic_, diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
    subs_command_freq->addTask(&command_freq_);  // Adding an additional task to the control

    shutting_down_ = false;

    bReceiving_ = false;

    cycle_freq_ = 0.0;
  }

  /*!	\fn RSComponent::checkCommandSubscriber
   * 	\brief Checks that the controller is receiving at a correct frequency the command messages. Diagnostics
  */
  void checkCommandSubscriber(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    ros::Time current_time = ros::Time::now();

    double diff = (current_time - last_command_time_).toSec();

    if (diff > 0.25)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
      // ROS_INFO("checkCommandSubscriber: cmd %lf seconds without commands", diff);
      // If this happens in safe velocity, deactivate flag in order to stop motors.
      // This condition is not relevant in position or position velocity commands
      bReceiving_ = false;
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
    }
  }

  /*!	\fn RSComponent::connectTest()
   * 	\brief Test
  */
  void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Connection test or ping test
    // IF OK
    status.summary(0, "Connected successfully.");
  }

  /*!	\fn RSComponent::controllerDiagnostic
   * 	\brief Checks the status of the driver Diagnostics
  */
  void controllerDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    // add and addf are used to append key-value pairs.
    // stat.addf("Controller StatusWord (HEX)", "%x", sw ); // Internal controller status

    // TODO => Add diagnostic info
    // stat.add("Encoder position left", (int) position_left_pps_ );
    // stat.add("Encoder position right", (int) position_right_pps_ );
  }

  /*!	\fn RSComponent::~RSComponent()
   * 	\brief Public destructor
  */
  ~RSComponent()
  {
    delete subs_command_freq;
  }

  /*!	\fn int RSComponent::start()
   * 	\brief Starts the component
  */
  int start()
  {
    freq_diag_.clear();
    running = true;
    return 0;
  }

  /*!	\fn int RSComponent::stop()
   * 	\brief Stops the component
  */
  int stop()
  {
  }

  /*! \fn read_and_publish
   *  Do some stuff and publishes state/messages to ROS
   */
  int read_and_publish()
  {
    // Time measurement
    ros::Time current_time = ros::Time::now();
    double diff = (current_time - last_read_time_).toSec();
    last_read_time_ = current_time;
    if (diff > 0.0)
      cycle_freq_ = 1.0 / diff;

    /* EXAMPLE */
    // Publish message
    // joint_states_.header.stamp = ros::Time::now();
    // joint_state_pub_.publish( joint_states_ );
  }

  /*! \fn bool spin()
    *  Main Loop
  */
  bool spin()
  {
    ros::Rate r(desired_freq_);

    while (!(shutting_down_ =
                 ros::isShuttingDown()))  // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
        while (ros::ok() && node_handle_.ok())
        {
          read_and_publish();

          self_test_.checkTest();

          diagnostic_.update();

          ros::spinOnce();
          r.sleep();
        }

        RCOMPONENT_INFO("END OF ros::ok() !!!");
      }
      else
      {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    return true;
  }

  /* EXAMPLES */
  /*! \fn  RSComponent::cmdJointStateCallback
    * Receive joint references
  */
  /*void cmdJointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {

      return;
  }*/

  /*! \fn  Service Stop
    * Stop robot
  */
  /*bool srvCallbackStop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
      ROS_INFO("RSComponent:: STOP");

    this->stop();

    return true;
  }*/

};  // class RSComponent

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rscomponent");

  ros::NodeHandle n;
  RSComponent ctrl(n);

  ctrl.spin();

  return (0);
}
// EOF
