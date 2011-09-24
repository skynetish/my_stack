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
#include <cstdio>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSource::TeleopSource(TeleopSourceCallback callback)
  : mCallback(callback) {
}
//=============================================================================
TeleopSource::~TeleopSource() {
  //Stop listening thread and block until it finishes
  stop(true);
}
//=============================================================================
bool TeleopSource::start(bool blocking) {
  //Check if running
  if (isRunning()) {
    return true;
  }

  //Prepare to listen
  if (!prepareToListen()) {
    printf("TeleopSource::start: error in prepareToListen()\n");
    return false;
  }

  //Create thread which executes listen loop
  mThread = boost::thread(&TeleopSource::listenLoop, this);

  //If blocking wait for thread to finish
  if (blocking) {
    mThread.join();
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::isRunning() {
  //Check if thread has same ID as default thread (which is "Not-A-Thread")
  return (boost::thread::id() != mThread.get_id());
}
//=============================================================================
bool TeleopSource::stop(bool blocking) {
  //Check if running
  if (!isRunning()) {
    return true;
  }

  //Interrupt
  mThread.interrupt();

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
void TeleopSource::listenLoop() {
  TeleopState teleopState;  //latest teleop state
  int listenResult;         //listen result
  bool success = true;      //return value

  //Loop until interrupted
  while (!boost::this_thread::interruption_requested()) {

    //Listen for events
    listenResult = listen(TIMEOUT_SECONDS, &teleopState);

    //Deal with result
    switch (listenResult) {
      case LISTEN_ERROR:
        //Error
        printf("TeleopSource::listenLoop: error in listen()\n");
        success = false;
        break;
      case LISTEN_STATE_UNCHANGED:
        //Do nothing this time around
        break;
      case LISTEN_STATE_CHANGED:
        //Enforce threshold
        for (size_t i = 0; i < teleopState.axes.size(); i++) {
          if (AXIS_THRESHOLD > fabs(teleopState.axes[i].value)) {
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




//=============================================================================
} //namespace
//=============================================================================
