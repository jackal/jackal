/**
Software License Agreement (BSD)

\file      joy_pwm_node.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "jackal_msgs/Drive.h"
#include "sensor_msgs/Joy.h"

#include "boost/algorithm/clamp.hpp"

namespace jackal_teleop
{

class SimpleJoy
{
public:
  SimpleJoy(ros::NodeHandle* nh);
  void init();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timeoutCallback(const ros::TimerEvent&);

private:
  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher drive_pub_;
  ros::Timer timeout_timer_;

  int deadman_button_;
  int axis_linear_;
  int axis_angular_;
  float scale_linear_;
  float scale_angular_;
};

SimpleJoy::SimpleJoy(ros::NodeHandle* nh) : nh_(nh)
{
}

void SimpleJoy::init()
{
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("joy", 1, &SimpleJoy::joyCallback, this);
  drive_pub_ = nh_->advertise<jackal_msgs::Drive>("cmd_drive", 1, true);
  timeout_timer_ = nh_->createTimer(ros::Duration(0), &SimpleJoy::timeoutCallback, this, true);

  ros::param::param("~deadman_button", deadman_button_, 7);
  ros::param::param("~axis_linear", axis_linear_, 0);
  ros::param::param("~axis_angular", axis_angular_, 1);
  ros::param::param("~scale_linear", scale_linear_, 1.0f);
  ros::param::param("~scale_linear", scale_angular_, 1.0f);
}

void SimpleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // New message arrived, stop the previous timeout timer.
  timeout_timer_.stop();

  if (joy_msg->buttons[deadman_button_])
  {
    jackal_msgs::Drive drive_msg;
    drive_msg.mode = jackal_msgs::Drive::MODE_PWM;
    float linear = joy_msg->axes[axis_linear_] * scale_linear_;
    float angular = joy_msg->axes[axis_angular_] * scale_angular_;
    drive_msg.drivers[jackal_msgs::Drive::LEFT] = boost::algorithm::clamp(linear - angular, -1.0, 1.0);
    drive_msg.drivers[jackal_msgs::Drive::RIGHT] = boost::algorithm::clamp(linear + angular, -1.0, 1.0);
    drive_pub_.publish(drive_msg);

    // If there hasn't been another Joy message in 100ms, time out and transmit
    // a zero velocity message.
    timeout_timer_.setPeriod(ros::Duration(0.1));
  }
  else
  {
    // Immediately time out and transmit a zero-motion message.
    timeout_timer_.setPeriod(ros::Duration());
  }

  // Start the timeout ticking.
  timeout_timer_.start();
}

void SimpleJoy::timeoutCallback(const ros::TimerEvent&)
{
  jackal_msgs::Drive drive_msg;
  drive_msg.mode = jackal_msgs::Drive::MODE_NONE;
  drive_pub_.publish(drive_msg);
}

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jackal_teleop_joy_pwm");
  ros::NodeHandle nh;
  jackal_teleop::SimpleJoy simple_joy(&nh);
  simple_joy.init();

  ros::spin();
}
