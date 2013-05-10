/*
*
* Copyright (c) 2013, BMW Car IT GmbH
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the <organization> nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "ros/ros.h"
#include "chatter_msgs/Chatter.h"

ros::Publisher pub;

void timer_expired(const ros::TimerEvent& evt) {
  chatter_msgs::Chatter msg;
  msg.header.stamp = ros::Time::now();
  msg.sender = ros::this_node::getName();

  std::string msg_string("Hello World");
  if( !ros::param::get("~msg_string", msg_string) ) {
    //ROS_WARN("No msg_string parameter defined, using default value.");
  }

  msg.message = msg_string;
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chatter_sender");
  ros::NodeHandle nh;

  double frequency = 0.02;
  if( !ros::param::get("~send_frequency", frequency) ) {
    //ROS_WARN("No send_frequency parameter defined, using default value.");
  }
  ros::Timer timer = nh.createTimer(ros::Duration(frequency), &timer_expired, false, true);
  pub = nh.advertise<chatter_msgs::Chatter>("Chatter", 1);
  ros::spin();

  return 0;
}
