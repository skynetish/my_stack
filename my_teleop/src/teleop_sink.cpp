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
#include <ros/ros.h>
#include <unistd.h>
#include <cstdio>
#include <signal.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>


#define KEYCODE_UP              0x41
#define KEYCODE_DOWN            0x42
#define KEYCODE_RIGHT           0x43
#define KEYCODE_LEFT            0x44
#define KEYCODE_SPACE           0x20

#define PARAM_KEY_TOPIC         "topic"
#define PARAM_KEY_USE_STEPS     "use_steps"
#define PARAM_KEY_NUM_STEPS     "num_steps"

#define PARAM_DEFAULT_TOPIC     "teleop_source"
#define PARAM_DEFAULT_USE_STEPS false
#define PARAM_DEFAULT_NUM_STEPS 10


class TeleopSourceKeyboard
{
public:
  TeleopSourceKeyboard();
  int run();

private:
  ros::NodeHandle* nh_;
  ros::Publisher pub_;
  geometry_msgs::Twist twist_;
  std::string topic_;
  bool use_steps_;
  int num_steps_;
};

TeleopSourceKeyboard::TeleopSourceKeyboard()
{
  //Create node handle using private namespace
  nh_ = new ros::NodeHandle("~");

  //Read parameters and set default values
  nh_->param(PARAM_KEY_TOPIC, topic_, std::string(PARAM_DEFAULT_TOPIC));
  nh_->param(PARAM_KEY_USE_STEPS, use_steps_, PARAM_DEFAULT_USE_STEPS);
  nh_->param(PARAM_KEY_NUM_STEPS, num_steps_, PARAM_DEFAULT_NUM_STEPS);

  //Advertise parameters for introspection
  nh_->setParam(PARAM_KEY_TOPIC, topic_);
  nh_->setParam(PARAM_KEY_USE_STEPS, use_steps_);
  nh_->setParam(PARAM_KEY_NUM_STEPS, num_steps_);

  //Advertise publisher
  pub_ = nh_->advertise<geometry_msgs::Twist>(topic_, 1);
}

int TeleopSourceKeyboard::run()
{
  //Print welcome message
  printf("Use arrow keys to move and space to stop...\n");

  //Loop and wait for key events
  while(1)
  {
    //Read from stdin
    char c;
    if(read(STDIN_FILENO, &c, 1) < 0)
    {
      printf("Error while reading\n");
      return 1;
    }

    //Feedback
    printf("  read: 0x%02X\n", c);

    //Handle important keys
    switch(c)
    {
      case KEYCODE_LEFT:
        printf("LEFT\n");
        twist_.linear.y= 1.0;
        break;
      case KEYCODE_RIGHT:
        printf("RIGHT\n");
        twist_.linear.y = -1.0;
        break;
      case KEYCODE_UP:
        printf("UP\n");
        twist_.linear.x = 1.0;
        break;
      case KEYCODE_DOWN:
        printf("DOWN\n");
        twist_.linear.x = -1.0;
        break;
      case KEYCODE_SPACE:
        printf("SPACE\n");
        twist_.linear.x = 0.0;
        twist_.linear.y = 0.0;
        twist_.linear.z = 0.0;
        twist_.angular.x = 0.0;
        twist_.angular.y = 0.0;
        twist_.angular.z = 0.0;
        break;
    }

    //TODO: print result
    //...

    //Publish result
    pub_.publish(twist_);
  }
}


struct termios oldTermios, rawTermios;


void quit(int sig)
{
  tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  //Update the standard input to use raw mode
  tcgetattr(STDIN_FILENO, &oldTermios);
  memcpy(&rawTermios, &oldTermios, sizeof(struct termios));
  rawTermios.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &rawTermios);

  //Initialise ROS
  ros::init(argc, argv, "teleop_source_keyboard");

  //Create teleop
  TeleopSourceKeyboard teleop;

  //Handle interrupt signal
  signal(SIGINT,quit);

  //Run teleop and return result
  int result = teleop.run();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios);
  return result;
}
