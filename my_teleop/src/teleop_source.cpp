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
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <boost/thread.hpp>
#include <cstdio>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSource::TeleopSource(TeleopSourceCallback callback)
  : mCallback(callback), mListenTimeout(LISTEN_TIMEOUT_DEFAULT) {
  //Initialise array members
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = AXIS_DEAD_ZONE_DEFAULT;
    mAxisInverted[i] = false;
  }
}
//=============================================================================
TeleopSource::~TeleopSource() {
  //Stop listening thread and block until it finishes
  stop(true);
}
//=============================================================================
bool TeleopSource::start(bool blocking) {
  //Lock changes to thread state
  boost::unique_lock<boost::recursive_mutex> mutexLock(mMutex);

  //Check if running
  if (!isRunning()) {
    //Prepare to listen
    if (!prepareToListen()) {
      printf("TeleopSource::start: error in prepareToListen()\n");
      return false;
    }

    //Create thread which executes listen loop
    mThread = boost::thread(&TeleopSource::listenLoop, this);
  }

  //Unlock changes to thread state so we can safely block
  mutexLock.unlock();

  //If blocking wait for thread to finish
  if (blocking) {
    mThread.join();
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::stop(bool blocking) {
  //Lock changes to thread state
  boost::unique_lock<boost::recursive_mutex> mutexLock(mMutex);

  //Check if running
  if (!isRunning()) {
    return true;
  }

  //Interrupt
  mThread.interrupt();

  //Unlock changes to thread state so we can safely block
  mutexLock.unlock();

  //If blocking wait for thread to finish
  if (blocking) {
    mThread.join();
  }

  //Done listening
  if (!doneListening()) {
    printf("TeleopSource::stop: error in doneListening()\n");
    return false;
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::isRunning() {
  //Lock changes to thread state
  boost::unique_lock<boost::recursive_mutex> mutexLock(mMutex);

  //Check if thread has same ID as default thread (which is "Not-A-Thread")
  return (boost::thread::id() != mThread.get_id());
}
//=============================================================================
void TeleopSource::listenLoop() {
  TeleopState teleopState;  //latest teleop state
  int listenResult;         //listen result
  bool success = true;      //return value

  //Loop until interrupted
  while (success && !boost::this_thread::interruption_requested()) {

    //Listen for events
    listenResult = listen(mListenTimeout, &teleopState);

    //Deal with result
    switch (listenResult) {
      case LISTEN_RESULT_ERROR:
        //Error
        printf("TeleopSource::listenLoop: error in listen()\n");
        success = false;
        break;
      case LISTEN_RESULT_UNCHANGED:
        //Do nothing this time around
        break;
      case LISTEN_RESULT_CHANGED:
        //Enforce axis inversion and dead zone
        for (size_t i = 0; i < teleopState.axes.size(); i++) {
          if (mAxisInverted[teleopState.axes[i].type]) {
            teleopState.axes[i].value *= -1.0;
          }
          if (mAxisDeadZone[teleopState.axes[i].type] > fabs(teleopState.axes[i].value)) {
            teleopState.axes[i].value = 0.0;
          }
        }

        //Call callback
        success = mCallback(&teleopState);
        if (!success) {
          printf("TeleopSource::listenLoop: error in callback\n");
        }
        break;
      default:
        //Invalid result
        printf("TeleopSource::listenLoop: invalid result from listen() (%d)\n", listenResult);
        success = false;
        break;
    }
  }

  //When done, zero all outputs
  for (int i=0; i<(int)teleopState.axes.size(); i++) {
    teleopState.axes[i].value = 0.0;
  }
  for (int i=0; i<(int)teleopState.buttons.size(); i++) {
    teleopState.buttons[i].value = 0;
  }
}
//=============================================================================
bool TeleopSource::setListenTimeout(int listenTimeout) {
  if (isRunning()) {
    printf("TeleopSource::setListenTimeout: cannot be done while thread is running\n");
    return false;
  }
  if (LISTEN_TIMEOUT_MIN > listenTimeout || LISTEN_TIMEOUT_MAX < listenTimeout) {
    printf("TeleopSource::setListenTimeout: invalid listen timeout (%d)\n", listenTimeout);
    return false;
  }
  mListenTimeout = listenTimeout;
  return true;
}
//=============================================================================
int TeleopSource::getListenTimeout() {
  return mListenTimeout;
}
//=============================================================================
bool TeleopSource::setAxisDeadZoneForAllAxes(float axisDeadZone) {
  if (isRunning()) {
    printf("TeleopSource::setListenTimeout: cannot be done while thread is running\n");
    return false;
  }
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    printf("TeleopSource::setAxisDeadZoneForAllAxes: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = axisDeadZone;
  }
  return true;
}
//=============================================================================
bool TeleopSource::setAxisDeadZone(float axisDeadZone, TeleopAxisType axisType) {
  if (isRunning()) {
    printf("TeleopSource::setListenTimeout: cannot be done while thread is running\n");
    return false;
  }
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    printf("TeleopSource::setAxisDeadZone: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }
  mAxisDeadZone[axisType] = axisDeadZone;
  return true;
}
//=============================================================================
float TeleopSource::getAxisDeadZone(TeleopAxisType axisType) {
  return mAxisDeadZone[axisType];
}
//=============================================================================
bool TeleopSource::setAxisInvertedForAllAxes(bool axisInverted) {
  if (isRunning()) {
    printf("TeleopSource::setListenTimeout: cannot be done while thread is running\n");
    return false;
  }
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisInverted[i] = axisInverted;
  }
  return true;
}
//=============================================================================
bool TeleopSource::setAxisInverted(bool axisInverted,
                                   TeleopAxisType axisType) {
  if (isRunning()) {
    printf("TeleopSource::setListenTimeout: cannot be done while thread is running\n");
    return false;
  }
  mAxisInverted[axisType] = axisInverted;
  return true;
}
//=============================================================================
bool TeleopSource::getAxisInverted(TeleopAxisType axisType) {
  return mAxisInverted[axisType];
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
