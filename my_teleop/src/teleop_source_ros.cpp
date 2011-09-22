/*
 * Copyright (c) 2011, Kevin LeBlanc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




//=============================================================================
//Includes
//=============================================================================
#include <cstdio>
#include <signal.h>
#include <ros/ros.h>
#include <my_teleop/State.h>
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <teleop_source_keyboard.hpp>
#include <teleop_source_joystick.hpp>




//=============================================================================
//Defines
//=============================================================================

/**@{ Parameter keys */
#define PARAM_KEY_TOPIC            "topic"
#define PARAM_KEY_TELEOP_TYPE      "type"
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TOPIC        "teleop"
#define PARAM_DEFAULT_TELEOP_TYPE  "keyboard"
/**@}*/




//=============================================================================
//Globals
//=============================================================================
/** Publisher must be global to allow it to be used in callback */
ros::Publisher* gPublisher = NULL;

/** Teleop source must be global to allow it to be used in signal handler */
teleop::TeleopSource* gTeleopSource = NULL;


//=============================================================================
//Function prototypes
//=============================================================================

/**
 * Interrupt signal handler
 *
 *   @param sig - received signal
 */
void quit(int sig);

/**
 * Teleop source callback for receiving teleop state
 *
 *   @param teleopState - latest teleop state
 */
bool teleopSourceCallback(teleop::TeleopState* teleopState);

/**
 * Utility function for creating appropriate teleop source
 *
 *   @param teleopType - string containing teleop type
 */
teleop::TeleopSource* teleopSourceFactory(std::string teleopType);




//=============================================================================
//Function definitions
//=============================================================================
void quit(int sig) {
  //Stop the teleop source
  if (NULL != gTeleopSource) {
    gTeleopSource->stop(true);
  }

  //Shutdown ROS to allow us to exit
  ros::shutdown();
}
//=============================================================================
bool teleopSourceCallback(teleop::TeleopState* teleopState) {
  //Sanity check publisher and teleopState
  if (NULL == gPublisher || NULL == teleopState) {
    return false;
  }

  //Convert from teleop::teleopState to teleop_msgs::State
  my_teleop::State teleopStateMsg;
  //TODO

  //Publish result
  gPublisher->publish(teleopStateMsg);

  //Return result
  return true;
}
//=============================================================================
teleop::TeleopSource* teleopSourceFactory(std::string* teleopType) {
  if (teleopType->compare("keyboard")) {
    return new teleop::TeleopSourceKeyboard(&teleopSourceCallback);
  } else if (teleopType->compare("joystick")) {
    return new teleop::TeleopSourceJoystick(&teleopSourceCallback);
  }
  return NULL;
}




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv)
{
  //Initialise ROS (exceptions ignored intentionally)
  ros::init(argc, argv, "teleop_source_ros", ros::init_options::NoSigintHandler);

  //Set interrupt handler
  signal(SIGINT, quit);

  //Node handle uses private namespace (exceptions ignored intentionally)
  ros::NodeHandle nodeHandle("~");

  //Declare parameters
  std::string topic;
  std::string teleopType;

  //Read parameters and set default values
  nodeHandle.param(PARAM_KEY_TOPIC,       topic,      std::string(PARAM_DEFAULT_TOPIC));
  nodeHandle.param(PARAM_KEY_TELEOP_TYPE, teleopType, std::string(PARAM_DEFAULT_TELEOP_TYPE));

  //Advertise parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC, topic);
  nodeHandle.setParam(PARAM_KEY_TOPIC, teleopType);

  //Advertise publisher with buffer size set to 1 and latching on.  The
  //publisher should basically just always contain the latest teleop state.
  ros::Publisher publisher = nodeHandle.advertise<my_teleop::State>(topic, 1, true);
  gPublisher = &publisher;

  //Create teleop source of the given type
  gTeleopSource = teleopSourceFactory(&teleopType);
  if (NULL == gTeleopSource) {
    return 1;
  }

  //Start teleop source in non-blocking mode so we can spin for ROS events
  gTeleopSource->start(false);

  //Spin until we're done
  ros::spin();

  return 0;
}
