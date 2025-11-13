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
 * @file bag_recorder.cpp
 * @brief Defines the SimpleBagRecorder class to record all topics while talker
 * is running
 * @author Daniel Zinobile
 * @date 12-Nove-2025
 */

#include "rcl_interfaces/msg/log.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
namespace fs = std::filesystem; // For searching file system

/**
 * @class SimpleBagRecorder
 * @brief Modified from tutorial example, records from all topics while talker
 * is running
 */
class SimpleBagRecorder : public rclcpp::Node {
public:
  /**
   * @brief Constructor for SimpleBagRecorder
   */
  SimpleBagRecorder() : Node("simple_bag_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>(); // writer to ros2 bag

    // Increment name of my_bag directory to prevent overwriting
    int counter = 1;
    std::string bag_path;
    do {
      bag_path = "results/my_bag" + std::to_string(counter);
      counter++;
    } while (fs::exists(bag_path));
    writer_->open(bag_path);

    RCLCPP_INFO_STREAM(get_logger(),
                       "Recording to bag: %s" << bag_path.c_str());

    // Create subscriptions for each available topic
    chatter_subscription_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        std::bind(&SimpleBagRecorder::chatter_callback, this, _1));

    parameter_events_subscription_ =
        create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "parameter_events", 10,
            std::bind(&SimpleBagRecorder::parameter_events_callback, this, _1));

    rosout_subscription_ = create_subscription<rcl_interfaces::msg::Log>(
        "rosout", 10, std::bind(&SimpleBagRecorder::rosout_callback, this, _1));

    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "tf", 10, std::bind(&SimpleBagRecorder::tf_callback, this, _1));
  }

private:
  /**
   * @brief Callback function for subscription to /chatter topic
   * @param msg Message from /chatter topic
   */
  void chatter_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }

  /**
   * @brief Callback function for subscription to /parameter_events topic
   * @param msg Message from /parameter_events topic
   */
  void parameter_events_callback(
      std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "parameter_events",
                   "rcl_interfaces::msg::ParameterEvent", time_stamp);
  }

  /**
   * @brief Callback function for subscription to /rosout topic
   * @param msg Message from /rosout topic
   */
  void rosout_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "rosout", "rcl_interfaces::msg::Log", time_stamp);
  }

  /**
   * @brief Callback function for subscription to /tf topic
   * @param msg Message from /tf topic
   */
  void tf_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "tf", "tf2_msgs::msg::TFMessage", time_stamp);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      parameter_events_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr
      rosout_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}