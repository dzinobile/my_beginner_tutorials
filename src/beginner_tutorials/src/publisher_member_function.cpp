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

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/find_difference.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), fib_a_(0), fib_b_(1) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    service_ = this->create_service<beginner_tutorials::srv::FindDifference>(
        "subtract_two_ints",
        std::bind(&MinimalPublisher::subtract, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

 private:
  long long next_fib() {
    long long current = fib_a_;
    if (fib_b_ > std::numeric_limits<long long>::max() - fib_a_) {
      fib_a_ = 0;
      fib_b_ = 1;
      RCLCPP_FATAL_STREAM(this->get_logger(), "Overflow: restarting sequence");
      return current;
    }

    long long next = fib_a_ + fib_b_;
    fib_a_ = fib_b_;
    fib_b_ = next;

    return current;
  }
  void timer_callback() {
    auto message = std_msgs::msg::String();
    long long value = next_fib();
    message.data = "Fibonacci sequence: " + std::to_string(value);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());
    publisher_->publish(message);
    if (value < 1000000) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Number less than 1000000000");
    } else if (value >= 1000000 && value < 1000000000000000) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "1000000 < Number < 1000000000000000");
    } else if (1000000000000000 <= value) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Approaching upper limit for int64");
    }
  }
  void subtract(
      const std::shared_ptr<beginner_tutorials::srv::FindDifference::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::FindDifference::Response>
          response) {
    response->difference = request->b - request->a;
    auto server_message = std_msgs::msg::String();
    server_message.data = std::to_string(response->difference);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
                (long int)response->difference);
    publisher_->publish(server_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::FindDifference>::SharedPtr service_;
  long long fib_a_;
  long long fib_b_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
