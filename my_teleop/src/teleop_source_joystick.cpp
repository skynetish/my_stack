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
    mNumAxes(0), mNumButtons(0) {
  for (int i = 0; i < ABS_CNT; i++) {
    mAxisMap[i] = ABS_MISC;
  }
  for (int i = 0; i < KEY_MAX - BTN_MISC + 1; i++) {
    mButtonMap[i] = BTN_MISC;
  }
}
//=============================================================================
bool TeleopSourceJoystick::prepareToListen() {
  //Open device in non-block mode
  mFileDescriptor = open(mDevice.c_str(), O_RDONLY | O_NONBLOCK);
  if (-1 == mFileDescriptor) {
    printf("TeleopSourceJoystick::prepareToListen: error opening device\n");
    return false;
  }

  //Get number of axes and buttons and corresponding maps
  if (-1 == ioctl(mFileDescriptor, JSIOCGAXES, &mNumAxes)) {
    printf("TeleopSourceJoystick::prepareToListen: error reading number of axes\n");
    return false;
  }
  if (-1 == ioctl(mFileDescriptor, JSIOCGAXMAP, mAxisMap)) {
    printf("TeleopSourceJoystick::prepareToListen: error reading axis map\n");
    return false;
  }
  if (-1 == ioctl(mFileDescriptor, JSIOCGBUTTONS, &mNumButtons)) {
    printf("TeleopSourceJoystick::prepareToListen: error reading number of buttons\n");
    return false;
  }
  if (-1 == ioctl(mFileDescriptor, JSIOCGBTNMAP, mButtonMap)) {
    printf("TeleopSourceJoystick::prepareToListen: error reading button map\n");
    return false;
  }

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

  //Resize and initialise teleopState axis and button vectors if needed
  if (mNumAxes != teleopState->axes.size()) {
    teleopState->axes.resize(mNumAxes);
    for (size_t i = 0; i < mNumAxes; i++) {
      teleopState->axes[i].type = axisEventTypeToTeleopType(mAxisMap[i]);
      teleopState->axes[i].value = 0.0;
    }
  }
  if (mNumButtons != teleopState->buttons.size()) {
    teleopState->buttons.resize(mNumButtons);
    for (size_t i = 0; i < mNumButtons; i++) {
      teleopState->buttons[i].type = buttonEventTypeToTeleopType(mButtonMap[i]);
      teleopState->buttons[i].value = 0;
    }
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
  int result = select(1, &fileDescriptorSet, NULL, NULL, &timeout);
  if (0 == result) {
    //Timeout
    return LISTEN_STATE_UNCHANGED;
  } else if (-1 == result) {
    //Error
    printf("TeleopSourceJoystick::listen: error in select() (%d)\n", errno);
    return LISTEN_ERROR;
  }

  //Data available, read and process all events until queue is empty
  bool stateChanged = false;
  js_event event;
  while (true) {

    //Read one event and check result
    ssize_t numBytes = read(mFileDescriptor, &event, sizeof(js_event));
    if ((-1 == numBytes) && (EAGAIN == errno)) {
      //Queue is empty
      if (stateChanged) {
        return LISTEN_STATE_CHANGED;
      } else {
        return LISTEN_STATE_UNCHANGED;
      }
    } else if (sizeof(js_event) == numBytes) {
      //Handle read event
      switch(handleEvent(&event, teleopState)) {
        case LISTEN_STATE_UNCHANGED:
          break;
        case LISTEN_STATE_CHANGED:
          stateChanged = true;
          break;
        case LISTEN_ERROR:
        default:
          return LISTEN_ERROR;
      }
    } else {
      //Error
      printf("TeleopSourceJoystick::listen: error in read()\n");
      return LISTEN_ERROR;
    }

  }
}
//=============================================================================
bool TeleopSourceJoystick::doneListening() {
  //Close joystick device if it was open
  if ((-1 != mFileDescriptor) && (0 != close(mFileDescriptor))) {
    printf("TeleopSourceJoystick::doneListening: error closing joystick device\n");
    return false;
  }

  //Return success
  return true;
}
//=============================================================================
int TeleopSourceJoystick::handleEvent(js_event* event, TeleopState* teleopState) {
  //Handle known events
  switch(event->type)
  {
    case JS_EVENT_AXIS:
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      //Event number shouldn't be bigger than the vector
      if(event->number >= teleopState->axes.size()) {
        return LISTEN_ERROR;
      }

      //Set value for this event and signal update
      teleopState->axes[event->number].value = axisEventValueToTeleopValue(event->value);
      return LISTEN_STATE_CHANGED;
    case JS_EVENT_BUTTON:
    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      //Event number shouldn't be bigger than the vector
      if(event->number >= teleopState->buttons.size()) {
        return LISTEN_ERROR;
      }

      //Set value for this event and signal update
      teleopState->buttons[event->number].value = buttonEventValueToTeleopValue(event->value);
      return LISTEN_STATE_CHANGED;
  }

  //Return no change
  return LISTEN_STATE_UNCHANGED;
}
//=============================================================================
std::string TeleopSourceJoystick::getDefaultDevice() {
  return std::string("/dev/input/js0");
}
//=============================================================================
float TeleopSourceJoystick::axisEventValueToTeleopValue(int16_t value) {
  return (float)(value)/32767.0;
}
//=============================================================================
int TeleopSourceJoystick::buttonEventValueToTeleopValue(int16_t value) {
  return (int)(value);
}
//=============================================================================
int TeleopSourceJoystick::axisEventTypeToTeleopType(uint8_t type) {
  switch (type) {
    case ABS_X:         return TELEOP_AXIS_TYPE_LIN_X;
    case ABS_Y:         return TELEOP_AXIS_TYPE_LIN_Y;
    case ABS_Z:         return TELEOP_AXIS_TYPE_LIN_Y;
    case ABS_RX:        return TELEOP_AXIS_TYPE_ROT_X;
    case ABS_RY:        return TELEOP_AXIS_TYPE_ROT_Y;
    case ABS_RZ:        return TELEOP_AXIS_TYPE_ROT_Z;
    case ABS_TILT_X:    return TELEOP_AXIS_TYPE_ROT_X;
    case ABS_TILT_Y:    return TELEOP_AXIS_TYPE_ROT_Y;
    case ABS_GAS:
    case ABS_THROTTLE:  return TELEOP_AXIS_TYPE_THROTTLE;
  }
  return TELEOP_AXIS_TYPE_UNKNOWN;
}
//=============================================================================
int TeleopSourceJoystick::buttonEventTypeToTeleopType(uint16_t type) {
  switch (type) {
    case BTN_0:         return TELEOP_BUTTON_TYPE_0;
    case BTN_1:         return TELEOP_BUTTON_TYPE_1;
    case BTN_2:         return TELEOP_BUTTON_TYPE_2;
    case BTN_3:         return TELEOP_BUTTON_TYPE_3;
    case BTN_4:         return TELEOP_BUTTON_TYPE_4;
    case BTN_5:         return TELEOP_BUTTON_TYPE_5;
    case BTN_6:         return TELEOP_BUTTON_TYPE_6;
    case BTN_7:         return TELEOP_BUTTON_TYPE_7;
    case BTN_8:         return TELEOP_BUTTON_TYPE_8;
    case BTN_9:         return TELEOP_BUTTON_TYPE_9;
    case BTN_A:         return TELEOP_BUTTON_TYPE_A;
    case BTN_B:         return TELEOP_BUTTON_TYPE_B;
    case BTN_C:         return TELEOP_BUTTON_TYPE_C;
    case BTN_X:         return TELEOP_BUTTON_TYPE_X;
    case BTN_Y:         return TELEOP_BUTTON_TYPE_Y;
    case BTN_Z:         return TELEOP_BUTTON_TYPE_Z;
    case BTN_RIGHT:     return TELEOP_BUTTON_TYPE_RIGHT;
    case BTN_LEFT:      return TELEOP_BUTTON_TYPE_LEFT;
    case BTN_SELECT:    return TELEOP_BUTTON_TYPE_SELECT;
    case BTN_START:     return TELEOP_BUTTON_TYPE_START;
    case BTN_TRIGGER:   return TELEOP_BUTTON_TYPE_TRIGGER;
  }
  return TELEOP_BUTTON_TYPE_UNKNOWN;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
