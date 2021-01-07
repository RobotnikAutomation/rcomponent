/*! \class ?RCNode
 *  \file ?rc_node.h
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date ?year
 *  \brief ?brief
 *
 */

#ifndef _?RC_NODE_
#define _?RC_NODE_

#include <rcomponent/rcomponent.h>

// Insert here general includes:
#include <math.h>

// Insert here msg and srv includes:
#include <std_msgs/String.h>
#include <robotnik_msgs/StringStamped.h>

class ?RCNode : public rcomponent::RComponent
{
public:
  ?RCNode(ros::NodeHandle h);
  virtual ~?RCNode();

protected:
  /*** RComponent stuff ***/

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  //! Actions performed on init state
  virtual void initState();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();
  //! callback executed when moving to emergency state
  virtual void switchToEmergencyState();
  //! callback executed when moving to failure state
  virtual void switchToFailureState();
  //! control loop
  virtual void controlLoop();

  /* RComponent stuff !*/

  /* ROS Stuff */

  // Publishers

  //! To publish the basic information
  ros::Publisher ?rc_node_data_pub_;
  ros::Publisher ?rc_node_data_stamped_pub_;

  //! Subscribers
  ros::Subscriber example_sub_;
  string example_sub_name_; // Name of the example_sub_ topic

  /* ROS stuff !*/

  /* ?RCNode stuff */

  std_msgs::String ?rc_node_data_;

  /* ?RCNode stuff !*/


};

#endif  // _?RC_NODE_
