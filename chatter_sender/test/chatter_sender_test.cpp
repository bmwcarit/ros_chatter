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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>
#include "chatter_msgs/Chatter.h"
#include <stdio.h>

void receiveChatterCallback(const chatter_msgs::Chatter::ConstPtr& msg) {
  std::cerr << "Received message!" << std::endl;
  EXPECT_TRUE(msg->message == "Hello World");
}

TEST(chatter_sender, send_test) {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("Chatter", 1000, receiveChatterCallback);
  std::cerr << "Subscribed to topic Chatter!" << std::endl;

  std::cerr << "Waiting for Publishers ";
  while(sub.getNumPublishers() < 1){
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    std::cerr << ".";
  }
  std::cerr << " done" << std::endl;

  // Give the sender some time to send message
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

/*
* implementation of spinner thread
*/
inline void rosTestSpinner() {
  ROS_INFO("START SPINNING");
  ros::spin();
  ROS_INFO("STOP SPINNING");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chatter_sender_test");
  ros::NodeHandle nh;
  //ros::start();

  boost::thread rosThread(&rosTestSpinner);

  ROS_INFO("START TESTING");
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  ros::shutdown();
  ROS_INFO("STOP TESTING");

  rosThread.join();

  return result;
}
