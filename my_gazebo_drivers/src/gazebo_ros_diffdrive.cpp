/*
 *  Copyright (C) 2011 Kevin LeBlanc - modified
 *  Copyright (C) 2009 Tony Pratkanic - modified
 *  Copyright (C) 2003 Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <boost/bind.hpp>




#define KEY_COMMAND_TOPIC       "command_topic"
#define KEY_ODOMETRY_TOPIC      "odometry_topic"
#define KEY_SERVER_ID           "server_id"
#define KEY_THREADS             "threads"
#define KEY_COMMAND_QUEUE       "command_queue"
#define KEY_ODOMETRY_QUEUE      "odometry_queue"
#define KEY_UPDATE_PERIOD       "update_period" //seconds
#define KEY_MODEL_NAME          "model_name"
#define KEY_INTERFACE_NAME      "interface_name"
#define KEY_TF_PARENT           "tf_parent"
#define KEY_TF_CHILD            "tf_child"

#define DEFAULT_COMMAND_TOPIC   "cmd_vel"
#define DEFAULT_ODOMETRY_TOPIC  "odom"
#define DEFAULT_SERVER_ID       0
#define DEFAULT_THREADS         2
#define DEFAULT_COMMAND_QUEUE   100
#define DEFAULT_ODOMETRY_QUEUE  1
#define DEFAULT_UPDATE_PERIOD   0.01 //seconds
#define DEFAULT_MODEL_NAME      "robot_description"
#define DEFAULT_INTERFACE_NAME  "position_iface_0"
#define DEFAULT_TF_PARENT       "tf_parent"
#define DEFAULT_TF_CHILD        "base_link"




class DiffDrive {
public:
  libgazebo::PositionIface *posIface;
  ros::NodeHandle* rnhParams_;
  ros::NodeHandle* rnhTopics_;
  ros::Subscriber  sub_;
  ros::Publisher   pub_;

  void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    ROS_DEBUG_STREAM(
        " pos " << this->posIface
        <<    " x " << cmd_msg->linear.x
        <<    " y " << cmd_msg->linear.y
        <<    " z " << cmd_msg->angular.z);

    if (this->posIface) {
      this->posIface->Lock(1);
      this->posIface->data->cmdVelocity.pos.x = cmd_msg->linear.x;
      this->posIface->data->cmdVelocity.pos.y = cmd_msg->linear.y;
      this->posIface->data->cmdVelocity.yaw = -(cmd_msg->angular.z);
      this->posIface->Unlock();
    }
  }

  //Constructor
  explicit DiffDrive(std::string nodeName) {

    //Create required objects
    libgazebo::Client *client = new libgazebo::Client();
    libgazebo::SimulationIface *simIface = new libgazebo::SimulationIface();
    this->posIface = new libgazebo::PositionIface();
    this->rnhParams_ = new ros::NodeHandle("~");
    this->rnhTopics_ = new ros::NodeHandle("");

    //Parameters
    int serverId;
    int threads;
    int commandQueue;
    int odometryQueue;
    double updatePeriod;
    std::string modelName;
    std::string interfaceName;
    std::string commandTopic;
    std::string odometryTopic;
    std::string tfParent;
    std::string tfChild;

    //Load parameters and set defaults
    rnhParams_->param(std::string(KEY_COMMAND_TOPIC), commandTopic,
                      nodeName + std::string("/") + std::string(DEFAULT_COMMAND_TOPIC));
    rnhParams_->param(std::string(KEY_ODOMETRY_TOPIC), odometryTopic,
                      nodeName + std::string("/") + std::string(DEFAULT_ODOMETRY_TOPIC));
    rnhParams_->param(std::string(KEY_SERVER_ID), serverId, DEFAULT_SERVER_ID);
    rnhParams_->param(std::string(KEY_THREADS), threads, DEFAULT_THREADS);
    rnhParams_->param(std::string(KEY_COMMAND_QUEUE), commandQueue, DEFAULT_COMMAND_QUEUE);
    rnhParams_->param(std::string(KEY_ODOMETRY_QUEUE), odometryQueue, DEFAULT_ODOMETRY_QUEUE);
    rnhParams_->param(std::string(KEY_UPDATE_PERIOD), updatePeriod, DEFAULT_UPDATE_PERIOD);
    rnhParams_->param(std::string(KEY_MODEL_NAME), modelName, std::string(DEFAULT_MODEL_NAME));
    rnhParams_->param(std::string(KEY_INTERFACE_NAME), interfaceName, std::string(DEFAULT_INTERFACE_NAME));
    rnhParams_->param(std::string(KEY_TF_PARENT), tfParent, std::string(DEFAULT_TF_PARENT));
    rnhParams_->param(std::string(KEY_TF_CHILD), tfChild, std::string(DEFAULT_TF_CHILD));

    //Advertise final parameters
    rnhParams_->setParam(std::string(KEY_COMMAND_TOPIC), commandTopic);
    rnhParams_->setParam(std::string(KEY_ODOMETRY_TOPIC), odometryTopic);
    rnhParams_->setParam(std::string(KEY_SERVER_ID), serverId);
    rnhParams_->setParam(std::string(KEY_THREADS), threads);
    rnhParams_->setParam(std::string(KEY_COMMAND_QUEUE), commandQueue);
    rnhParams_->setParam(std::string(KEY_ODOMETRY_QUEUE), odometryQueue);
    rnhParams_->setParam(std::string(KEY_UPDATE_PERIOD), updatePeriod);
    rnhParams_->setParam(std::string(KEY_MODEL_NAME), modelName);
    rnhParams_->setParam(std::string(KEY_INTERFACE_NAME), interfaceName);
    rnhParams_->setParam(std::string(KEY_TF_PARENT), tfParent);
    rnhParams_->setParam(std::string(KEY_TF_CHILD), tfChild);

    //Connect to the gazebo server
    try {
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
    } catch (gazebo::GazeboError e) {
      ROS_DEBUG_STREAM("Gazebo error: Unable to connect to gazebo: " << e);
      return;
    }

    //Open the simulator interface
    try {
      simIface->Open(client, "default");
    } catch (gazebo::GazeboError e) {
      ROS_DEBUG_STREAM("Gazebo error: Unable to connect to the simulator interface: " << e);
      return;
    }

    //Open the position interface
    try {
      this->posIface->Open(client, modelName + "::" + interfaceName);
    } catch (std::string e) {
      ROS_DEBUG_STREAM("Gazebo error: Unable to connect to the position interface: " << e);
      return;
    }

    //Enable the motors
    this->posIface->Lock(1);
    this->posIface->data->cmdEnableMotors = 1;
    this->posIface->Unlock();

    //Subscribe to the command topic and publish the odometry topic
    this->sub_ = rnhTopics_->subscribe<geometry_msgs::Twist>(commandTopic, commandQueue, &DiffDrive::cmdVelCallBack,this);
    this->pub_ = rnhTopics_->advertise<nav_msgs::Odometry>(odometryTopic, odometryQueue);

    //Spawn threads
    ros::MultiThreadedSpinner s(threads);
    boost::thread spinner_thread( boost::bind( &ros::spin, s ) );

    //Create odometry message
    nav_msgs::Odometry odom;

    //Create transform broadcaster
    tf::TransformBroadcaster transform_broadcaster_;

    //Set update frequency
    ros::Duration d;
    d.fromSec(updatePeriod);

    while(ros::ok()) {
      if (this->posIface) {
        this->posIface->Lock(1);

        // duplicate pr2_odometry functionalities, broadcast
        // transforms from base_footprint to odom
        // and from base_link to base_footprint

        // get current time
        ros::Time current_time_ = ros::Time::now();

        // getting data for base_link to odom transform
        btQuaternion qt;
        qt.setRPY(this->posIface->data->pose.roll, this->posIface->data->pose.pitch, -(this->posIface->data->pose.yaw));
        btVector3 vt(this->posIface->data->pose.pos.x, this->posIface->data->pose.pos.y, this->posIface->data->pose.pos.z);
        tf::Transform base_link_to_odom(qt, vt);
        transform_broadcaster_.sendTransform(tf::StampedTransform(base_link_to_odom,ros::Time::now(), tfParent, tfChild));

        // publish odom topic
        odom.pose.pose.position.x = this->posIface->data->pose.pos.x;
        odom.pose.pose.position.y = this->posIface->data->pose.pos.y;

        gazebo::Quatern rot;
        rot.SetFromEuler(gazebo::Vector3(this->posIface->data->pose.roll,this->posIface->data->pose.pitch,this->posIface->data->pose.yaw));

        odom.pose.pose.orientation.x = rot.x;
        odom.pose.pose.orientation.y = rot.y;
        odom.pose.pose.orientation.z = rot.z;
        odom.pose.pose.orientation.w = rot.u;

        odom.twist.twist.linear.x = this->posIface->data->velocity.pos.x;
        odom.twist.twist.linear.y = this->posIface->data->velocity.pos.y;
        odom.twist.twist.angular.z = -(this->posIface->data->velocity.yaw);

        odom.header.frame_id = odometryTopic;

        odom.header.stamp = ros::Time::now();

        this->pub_.publish(odom);

        this->posIface->Unlock();
      }
      d.sleep();
    }
  }

  //Destructor
  ~DiffDrive() {
    delete this->rnhParams_;
    delete this->rnhTopics_;
  }
};

//Main
int main(int argc, char** argv) {
  //Initialise
  std::string nodeName = basename(argv[0]);
  ros::init(argc, argv, nodeName);

  //Create instance
  DiffDrive d(nodeName);

  //Done
  return 0;
}
