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
#include <teleop_source_keyboard.hpp>
#include <cstdio>
#include <unistd.h>
#include <termios.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceKeyboard::TeleopSourceKeyboard(TeleopSourceCallback callback,
                                           int steps)
  : TeleopSource(callback), mSteps(steps) {
  //Clamp steps
  if (STEPS_MIN > mSteps) {
    mSteps = STEPS_MIN;
  } else if (STEPS_MAX < mSteps) {
    mSteps = STEPS_MAX;
  }
}
//=============================================================================
bool TeleopSourceKeyboard::prepareToListen() {
  //Raw termios settings
  struct termios rawTermios;

  //Update the standard input to use raw mode
  tcgetattr(STDIN_FILENO, &mOldTermios);
  memcpy(&rawTermios, &mOldTermios, sizeof(struct termios));
  rawTermios.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &rawTermios);

  //Print welcome message
  std::printf("Use arrow keys to move and space to stop...\n");

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceKeyboard::listen(TeleopState* teleopState) {

  //Sanity check
  if (NULL == teleopState) {
    return false;
  }

  //Ensure state has correct number of axes and buttons
  if (2 != teleopState->axes.size()) {
    TeleopAxis teleopAxisEmpty = {0, 0.0};
    teleopState->axes.resize(2, teleopAxisEmpty);
    teleopState->axes[0].type  = TELEOP_AXIS_TYPE_LIN_X;
    teleopState->axes[0].value = 0.0;
    teleopState->axes[1].type  = TELEOP_AXIS_TYPE_LIN_Y;
    teleopState->axes[1].value = 0.0;
  }
  if (0 != teleopState->buttons.size()) {
    teleopState->buttons.clear();
  }

  //Read from stdin
  char c;
  if(read(STDIN_FILENO, &c, 1) < 0) {
    return false;
  }

  //DEBUG
  printf("  read: 0x%02X\n", c);

  //Handle important keys
  switch(c) {
    case KEYCODE_UP:
      printf("UP\n"); //DEBUG
      teleopState->axes[0].value += (TELEOP_AXIS_MAX/mSteps);
      if (teleopState->axes[0].value > TELEOP_AXIS_MAX) {
        teleopState->axes[0].value = TELEOP_AXIS_MAX;
      }
      break;
    case KEYCODE_DOWN:
      printf("DOWN\n"); //DEBUG
      teleopState->axes[0].value -= (TELEOP_AXIS_MAX/mSteps);
      if (teleopState->axes[0].value < TELEOP_AXIS_MIN) {
        teleopState->axes[0].value = TELEOP_AXIS_MIN;
      }
      break;
    case KEYCODE_LEFT:
      printf("LEFT\n"); //DEBUG
      teleopState->axes[1].value -= (TELEOP_AXIS_MAX/mSteps);
      if (teleopState->axes[1].value < TELEOP_AXIS_MIN) {
        teleopState->axes[1].value = TELEOP_AXIS_MIN;
      }
      break;
    case KEYCODE_RIGHT:
      printf("RIGHT\n"); //DEBUG
      teleopState->axes[1].value += (TELEOP_AXIS_MAX/mSteps);
      if (teleopState->axes[1].value > TELEOP_AXIS_MAX) {
        teleopState->axes[1].value = TELEOP_AXIS_MAX;
      }
      break;
    case KEYCODE_SPACE:
      printf("SPACE\n"); //DEBUG
      teleopState->axes[0].value = 0.0;
      teleopState->axes[1].value = 0.0;
      break;
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceKeyboard::doneListening() {
  //Restore stdin to old values
  tcsetattr(STDIN_FILENO, TCSANOW, &mOldTermios);

  //Return result
  return true;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
