// // Copyright 2025 Zinobile-Corp LLC
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#include <functional>
#include <memory>

#include "beginner_tutorials/srv/find_difference.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    this->declare_parameter("text_color","white");


  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) {
    std::string color = this->get_parameter("text_color").as_string();
    if (color == "red") msg_color = "\033[31m";
    else if (color == "green") msg_color = "\033[32m";
    else if (color == "yellow") msg_color = "\033[33m";
    else if (color == "blue") msg_color = "\033[34m";
    else if (color == "white") msg_color = "\033[37m";
    else msg_color = "\033[0m"; 
    const std::string reset = "\033[0m";
    RCLCPP_INFO_STREAM(this->get_logger(), msg_color <<"I heard: " << msg.data.c_str()<<reset);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string msg_color;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::string color = "white";
  if (argc > 1) {
    color = argv[1];
  }
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

