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
#include <teleop_source_joystick.hpp>
#include <linux/input.h>
#include <linux/joystick.h>
#include <cstdio>
#include <fcntl.h>



//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceJoystick::TeleopSourceJoystick(TeleopSourceCallback callback,
                                           std::string device)
  : TeleopSource(callback), mDevice(device), mFileDescriptor(-1),
    mNumAxes(0), mAxisMap(NULL), mNumButtons(0), mButtonMap(NULL) {
}
//=============================================================================
bool TeleopSourceJoystick::prepareToListen() {
  //Open device
  mFileDescriptor = open(mDevice.c_str(), O_RDONLY);
  if (-1 == mFileDescriptor) {
    printf("TeleopSourceJoystick::prepareToListen: error opening device\n");
    return false;
  }

  //Get number of axes and buttons and corresponding maps
  ioctl(mFileDescriptor, JSIOCGAXES, &mNumAxes);
  ioctl(mFileDescriptor, JSIOCGBUTTONS, &mNumButtons);
  ioctl(mFileDescriptor, JSIOCGAXMAP, mAxisMap);     //TODO: check this
  ioctl(mFileDescriptor, JSIOCGBTNMAP, mButtonMap);  //TODO: check this

  //Print welcome message
  char name[128];
  if (ioctl(mFileDescriptor, JSIOCGNAME(sizeof(name)), name) < 0) {
    strncpy(name, "Unknown", sizeof(name));
  }
  printf("Device file:       %s\n", mDevice.c_str());
  printf("Joystick type:     %s\n", name);

  //Return success
  return true;
}
//=============================================================================
int TeleopSourceJoystick::listen(int timeoutSeconds, TeleopState* teleopState) {
  //Sanity check
  if (NULL == teleopState) {
    printf("TeleopSourceJoystick::listen: NULL teleop state\n");
    return LISTEN_ERROR;
  }

  //Add file descriptor to file descriptor set
  fd_set fileDescriptorSet;
  FD_ZERO (&fileDescriptorSet);
  FD_SET (mFileDescriptor, &fileDescriptorSet);

  //Initialise timeout value in seconds for select
  struct timeval timeout;
  timeout.tv_sec = timeoutSeconds;
  timeout.tv_usec = 0;

  //Use select to see if anything shows up before timeout
  js_event event;
  int result = select(1, &fileDescriptorSet, NULL, NULL, &timeout);
  switch (result) {
    case 0:
      //Timeout
      return LISTEN_STATE_UNCHANGED;
    case -1:
      //Error
      printf("TeleopSourceJoystick::listen: error in select() (%d)\n", errno);
      return LISTEN_ERROR;
    default:
      //Data available
      if (0 >= read(mFileDescriptor, &event, sizeof(js_event))) {
        printf("TeleopSourceJoystick::listen: error in read()\n");
        return LISTEN_ERROR;
      }
  }

  //Setup teleopState axis and button vectors
  if (mNumAxes > teleopState->axes.size()) {
    teleopState->axes.resize(mNumAxes);
    for (size_t i = 0; i < mNumAxes; i++) {
      teleopState->axes[i].type = mAxisMap[i]; //TODO: convert to teleop type
      teleopState->axes[i].value = 0;
    }
  }
  if (mNumButtons > teleopState->buttons.size()) {
    teleopState->buttons.resize(mNumButtons);
    for (size_t i = 0; i < mNumButtons; i++) {
      teleopState->buttons[i].type = mButtonMap[i]; //TODO: convert to teleop type
      teleopState->buttons[i].value = 0;
    }
  }

  //Handle known events
  switch(event.type)
  {
    case JS_EVENT_AXIS:
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      //Event number shouldn't be bigger than the vector
      if(event.number >= teleopState->axes.size()) {
        return LISTEN_ERROR;
      }

      //Set value for this event and signal update
      teleopState->axes[event.number].value = event.value; //TODO: normalise value?
      return LISTEN_STATE_CHANGED;
    case JS_EVENT_BUTTON:
    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      //Event number shouldn't be bigger than the vector
      if(event.number >= teleopState->buttons.size()) {
        return LISTEN_ERROR;
      }

      //Set value for this event and signal update
      teleopState->buttons[event.number].value = event.value;
      return LISTEN_STATE_CHANGED;
  }

  //Return no change
  return LISTEN_STATE_UNCHANGED;
}
//=============================================================================
bool TeleopSourceJoystick::doneListening() {
  //Close joystick device if it was open
  if (-1 != mFileDescriptor && 0 != close(mFileDescriptor)) {
    printf("TeleopSourceJoystick::doneListening: error closing joystick device\n");
    return false;
  }

  //Return success
  return true;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
