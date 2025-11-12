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
 * @file publisher_member_function.cpp
 * @brief Defines MinimalPublisher class for minimal_publisher node
 * @author Daniel Zinobile
 * @date 07-Nov-2025
 */
#include "beginner_tutorials/srv/find_difference.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @class MinimalPublisher
 * @brief Creates a node that logs sequential fibonacci numbers to INFO_STREAM
 * and warning messages to the other loggers Also contains a server that
 * subtracts two given numbers
 */
class MinimalPublisher : public rclcpp::Node {
public:
  /**
   * @brief Constructor for MinimalPublisher class
   * @param fib_a_ Current fibonacci number
   * @param fib_b_ Next fibonacci number
   */
  MinimalPublisher() : Node("minimal_publisher"), fib_a_(0), fib_b_(1) {
    // Publisher for fibonacci numbers
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // timer to set publish rate
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::fib_callback, this));

    // Service for subtracting two numbers
    service_ = this->create_service<beginner_tutorials::srv::FindDifference>(
        "subtract_two_ints",
        std::bind(&MinimalPublisher::subtract, this, std::placeholders::_1,
                  std::placeholders::_2));
    
    // Fixed frame broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  /**
   * @brief Returns current fibonacci number, calculates next, and updates
   * fib_a_ and fib_b_
   */
  long long next_fib() {
    long long current = fib_a_;

    // Prevent overflow by restarting from zero once limit is reached
    if (fib_b_ > std::numeric_limits<long long>::max() - fib_a_) {
      fib_a_ = 0;
      fib_b_ = 1;
      RCLCPP_FATAL_STREAM(this->get_logger(), "Overflow: restarting sequence");
      return current;
    }

    // Update numbers
    long long next = fib_a_ + fib_b_;
    fib_a_ = fib_b_;
    fib_b_ = next;

    return current;
  }
  /**
   * @brief Publishes fibonacci numbers
   *
   * Publishes message with fibonacci number to "topic" topic
   * Logs publishing of message to INFO_STREAM
   * Logs other messages to different log levels depending on
   */
  void fib_callback() {
    auto message = std_msgs::msg::String();
    long long value = next_fib();
    message.data = "Fibonacci sequence: " + std::to_string(value);
    // Log publish action to INFO_STREAM
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());
    // Publish message
    publisher_->publish(message);

    // Log message to a certain log level depending on size of fibonacci number
    if (value < 1000000) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Number less than 1000000000");
    } else if (value >= 1000000 && value < 1000000000000000) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "1000000 < Number < 1000000000000000");
    } else if (1000000000000000 <= value) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Approaching upper limit for int64");
    }

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 1.0;
    t.transform.translation.z = 1.0;
    t.transform.rotation.x = 0.5;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.5;

    tf_broadcaster_->sendTransform(t);
  }
  /**
   * @brief Subtracts two server request numbers and publishes result to "topic"
   * topic
   * @param request Request sent to server
   * @param response Response from server
   */
  void subtract(
      const std::shared_ptr<beginner_tutorials::srv::FindDifference::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::FindDifference::Response>
          response) {

    // Load server response with difference between requests
    response->difference = request->b - request->a;

    // Create string message and load with server response
    std_msgs::msg::String server_message;
    server_message.data = std::to_string(response->difference);

    // Log activity to INFO
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
                (long int)response->difference);

    // Publish server message to "topic"
    publisher_->publish(server_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::FindDifference>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  long long fib_a_;
  long long fib_b_;
};

/**
 * @brief Main function to create and spin node
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
