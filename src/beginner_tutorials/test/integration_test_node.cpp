// Copyright 2025 Zinobile-Corp LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file integration_test_node.cpp
 * @brief Level 2 integration test file to test talker service call
 * @author Daniel Zinobile
 * @date 12-Nov-2025
 */
#include "beginner_tutorials/srv/find_difference.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

auto Logger = rclcpp::get_logger(""); // create an initial Logger

/**
 * @class MyTestsFixture
 * @brief Class to create integration test node
 */
class MyTestsFixture {
public:
  /**
   * @brief Constructor for MyTestsFixture class
   */
  MyTestsFixture() {
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode1");
    Logger = testerNode
                 ->get_logger(); // make sure message will appear in rqt_console

    testerNode->declare_parameter<double>(
        "test_duration"); // parameter for duration of the test

    // get test duration and log message
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }
  /**
   * @brief Destructor for MyTestsFixture class
   */
  ~MyTestsFixture() {}

protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

/**
 * @brief Test case to test talker FindDifference service
 * @param MyTestsFixture Test fixture object passed to test case
 */
TEST_CASE_METHOD(MyTestsFixture, "test talker server", "[service]") {

  // Client for FindDifference service
  auto client =
      testerNode->create_client<beginner_tutorials::srv::FindDifference>(
          "subtractTwoInts");
  RCLCPP_INFO_STREAM(Logger, "subtractTwoInts client created");

  // Wait for 1 second to find service
  rclcpp::Time start_time = rclcpp::Clock().now();
  bool service_found = false;
  rclcpp::Duration duration = 0s;
  RCLCPP_INFO_STREAM(Logger, "Performing Test...");

  while ((rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION) {
    if (client->wait_for_service(1s)) {
      service_found = true;
      break;
    } else {
      RCLCPP_INFO_STREAM(Logger, "Waiting for 'subtractTwoInts' service...");
    }
  }

  // Log message when service is found
  REQUIRE(service_found);
  RCLCPP_INFO_STREAM(Logger, "'subtractTwoInts' service is available");

  // Load request with a and b values
  auto request =
      std::make_shared<beginner_tutorials::srv::FindDifference::Request>();
  request->a = 7;
  request->b = 3;

  auto future = client->async_send_request(request);

  // Wait for the result and determine pass or fail
  if (rclcpp::spin_until_future_complete(
          testerNode, future, std::chrono::duration<double>(TEST_DURATION)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO_STREAM(
        Logger, "service call succeeded. result = " << response->difference);
    REQUIRE(response->difference == (request->b - request->a));
  } else {
    FAIL("service call timed out or failed");
  }
}