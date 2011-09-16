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
#ifndef INCLUDE_TELEOP_SOURCE_HPP
#define INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <ros/ros.h>




//=============================================================================
//Defines
//=============================================================================

/** Source types */
#define TELEOP_SOURCE_TYPE_DEFAULT        0
#define TELEOP_SOURCE_TYPE_UNKNOWN        1
#define TELEOP_SOURCE_TYPE_KEYBOARD       2
#define TELEOP_SOURCE_TYPE_JOYSTICK       3

/** Macro to determine source type name */
#define TELEOP_SOURCE_TYPE_NAME(x) (\
  ((x) == TELEOP_SOURCE_TYPE_DEFAULT      ) ? "TELEOP_SOURCE_TYPE_DEFAULT"    :\
  ((x) == TELEOP_SOURCE_TYPE_UNKNOWN      ) ? "TELEOP_SOURCE_TYPE_UNKNOWN"    :\
  ((x) == TELEOP_SOURCE_TYPE_KEYBOARD     ) ? "TELEOP_SOURCE_TYPE_KEYBOARD"   :\
  ((x) == TELEOP_SOURCE_TYPE_JOYSTICK     ) ? "TELEOP_SOURCE_TYPE_JOYSTICK"   :\
                                              "TELEOP_SOURCE_TYPE_UNDEFINED"  )




//=============================================================================
//Classes
//=============================================================================

/** Teleop namespace */
namespace teleop {


/**
 * This class provides a framework for generic handling of tele-operation
 * sources.  The static teleopSourceFactory() method creates a teleop source
 * instance of a given type.
 *
 * The start() method starts a loop (in a separate thread if non-blocking) in
 * which the pure virtual listen() method is called in each iteration.
 *
 * The listen() method must be implemented by a sub-class (i.e. a teleop source
 * type), and it should simply block until one or more teleop events are
 * detected.  Detected events are processed in the listen() method and the
 * resulting output is published on a given topic from within the start()
 * method's loop.
 */
class TeleopSource
{

public:

  /**
   * Static factory method to create a teleop source of a given type.
   */
  static TeleopSource TeleopSourceFactory(int type);

  /**
   * Start listening and publishing teleop device activity.
   *
   *   @param blocking [in] - true if this method should block
   *
   *   @return true on success
   */
  bool start(bool blocking);

  /**
   * Stop listening and publishing teleop device activity.  If start was called
   * in blocking mode, it returns.
   *
   *   @param reset [in] - true if this method should set outputs to zero
   *
   *   @return true on success
   */
  bool stop(bool reset);

  /**
   * Blocks while listening for teleop source device events.  When events occur,
   * updates the teleop output status via the teleop parameter and returns.
   *
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return true on success
   */
  virtual bool listen(std::vector<teleop_msgs::Teleop>* teleop) = 0;

private:
  ros::NodeHandle* nodeHandle_;
  ros::Publisher publisher_;
  std::string topic_;
  std::vector<teleop_msgs::Teleop> teleop_;

  /**
   * Internal function to carry out the main listen loop.
   */
  bool loop();

}; //class

} //namespace




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
