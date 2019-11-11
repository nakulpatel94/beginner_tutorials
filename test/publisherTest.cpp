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
 * @file publisherTest.cpp
 *
 * @author Nakul Patel
 *
 * @brief rostests for talker node
 *
 * @version 1
 *
 * @date 2019-11-10
 *
 * @section DESCRIPTION
 *
 * This is the .cpp file that uses certain unit tests from 
 * gtest framework to test the functionality of talker node.
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"


/**
 * @brief Test to check whether the servie is in existence
 * @param none
 * @return none
 */
TEST(TalkerNodeTest, testTalkerNodeService ) {
  ros::NodeHandle nh; 

  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>(
      "modify_string");
  
  /// to check for the service call
  bool checkExist = client.waitForExistence(ros::Duration(3));
  /// expect true macro for test
  EXPECT_TRUE(checkExist);
}


/**
 * @brief Test is the serice works as expected
 *
 * This test sets the input string, with the help of service and 
 * modifies the base string, then the EXPECT_STREQ macro tests 
 * whether they are equal
 *
 * @param none
 * @return none
 */
TEST(TalkerNodeTest, testTalkerNodeServiceMessage ) {
  ros::NodeHandle nh; 
  
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>(
      "modify_string");
  /// creating the object for service
  beginner_tutorials::changeString servObj;

  servObj.request.inputString = "testString";
  
  /// calling the service with client onject
  client.call(servObj.request, servObj.response);

  /// macro to check whether strings are equal  
  EXPECT_STREQ("testString", servObj.response.outputString.c_str());
}







