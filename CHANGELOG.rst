^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcomponent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.0 (2019-11-12)
------------------
* Fix checkTopicsHealth when a topic is not required
  - It was not working since in the AddTopic.. the flag was not being passed
* Save & Get state time transition
  - It can be useful to know when the last state transition took place
* Add default methods to execute during the state transitions
* Add the 'required' condition in the topic health monitors
  - Check topics health only takes in consideration the topics with required=true
* Clang format
* solved race condition, closes `#2 <https://github.com/RobotnikAutomation/rcomponent/issues/2>`_
* readParam works with default parameters of different type
* corrected typo in one macro
* python: Change class to be inherited
  The new structure will allow to build classes from RComponent easily
* Add Topic Health Monitor
  Class to make easier to check that a topic is being received and it's active.
  Integrated in RComponent as list of monitors that can be add and checked either
  all together or individually.
* adding try/catch in controlLoop when calculating real frequency
* rcomponent: added readParam method
  this method provides a way to check if parameters are set
  or not before they are read.
* rcomponent: added init methods
  act as delegate constructors
* added this accessor to log macros
* added tests
* rcomponent: Adding .h to cmakelists to visualize in IDE
* added constructor with no arguments
  this constructor is needed by the procedures interface
* added rcomponent_log macros
* change return type of getStateString to std::string
* added public method isRunning
* added methods to get the namespaces used by the component
* added constructors to set private namespace
* corrected race condition on nodehandle during destruction
* formatted using clang-format-3.8 and google style guide
* Add new compilation dependency with robotnik_msgs
* rcomponent.py: now node_name variable keeps the /
* method start()/asyncStart() do not start if the setup method returns error
* added getComponentName method
  to be used if a component includes another, so we can read its name
* added async execution of controlloop (with thread)
* added rcomponent namespace
* setting catkin_package tags
* compiling rcomponent as a library
* first commit
* Contributors: RomanRobotnik, jgomezRobotnik, jmapariciorobotnik, marbosjo
