/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Nakul Patel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file talker.cpp
 *
 * @author Nakul Patel
 *
 * @brief ROS publisher node
 *
 * @version 1
 *
 * @date 2019-11-06
 *
 * @section DESCRIPTION
 *
 * This is the .cpp file that implements a publisher(talker)
 * node which publishes the message to the topic. Here, this node
 * will send the custom message over the ROS system.
 *
 */
#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"

/// To initialize the baseString appearing on console
extern std::string baseString = "This is the basic tutorial for"
    " implementing ROS services.";

/**
 *  @brief modifies the base string
 *
 *  @param &req- request object containing the input string from client
 *
 *  @param &res- response object for the service
 *
 *  @return bool- indicating success/failure of service call
 *
 *  Service callback function that modifies the baseString variable
 *  with the inputString from Request object, and also
 *  set the response data with same inputString.
 */
bool modifyString(beginner_tutorials::changeString::Request &req,
                  beginner_tutorials::changeString::Response &res) {
  if (req.inputString.size() > 0) {
    baseString = req.inputString;
    /// Stuffing the response object data with incoming request data
    res.outputString = req.inputString;

    /// Log messages for various levels
    ROS_INFO_STREAM("Service has been called.");
    ROS_DEBUG_STREAM("Output string is:" << res.outputString);
    return true;
  } else {
    ROS_FATAL_STREAM("You must enter non-empty string");
    return false;
  }
}


/**
 *  @brief main function for talker node
 *
 *  @param argc-int count of arguments given through terminal
 *
 *  @param argv-char** array of character pointers listing the arguments
 *
 *  @return 0
 *
 * This function implements the publisher for publishing the message on topic
 * alongwith added functionalities of changing the publishing rate of the messages.
 * This can be done by passing the value for frequency from the roslaunch command.
 *
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   *
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  /// Variable for setting the publishing frequency/rate
  int freq = 10;

  /// decision for argument passed from command line
  if (argc == 2) {
    int clp = atoi(argv[1]);
    if (clp > 0) {
      freq = clp;
      ROS_INFO_STREAM("Input rate is now:" << freq);
    } else {
      ROS_ERROR_STREAM("Publishing rate is not valid, cannot be negative.");
    }
  } else {
    ROS_DEBUG_STREAM("Default publishing rate of 10 Hz used.");
  }


  /**
   * A ros::Rate object allows you to specify a
   * frequency that you would like to loop at.
   */
  ros::Rate loop_rate(freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  ros::ServiceServer service = n.advertiseService("modify_string",
                                                  modifyString);

  static tf::TransformBroadcaster br;

  tf::Transform transform;


  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    /**
     * This is stringstream object to contain a sequence of characters.
     * This sequence of characters can be accessed directly as a string object,
     *  using member str.
     */
    std::stringstream ss;

    ss << baseString << count;
    msg.data = ss.str();

    /**
     * Info level Log message that either contains the original baseString or
     * the baseString after modification with service call.
     *
     */
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);



    transform.setOrigin( tf::Vector3(10.0, 20.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 30);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    /**
     * using the ros::Rate object to sleep for the time remaining to let us
     * hit our publish rate.
     */
    loop_rate.sleep();
    ++count;
  }

  ROS_FATAL_STREAM("ROS node has been terminated.");

  return 0;
}

