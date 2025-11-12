#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <filesystem>

using std::placeholders::_1;
namespace fs = std::filesystem;
class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    int counter = 1;
    std::string bag_path;
    do {
        bag_path = "results/my_bag" + std::to_string(counter);
        counter++;
    } while(fs::exists(bag_path));

    writer_->open(bag_path);
    RCLCPP_INFO_STREAM(get_logger(), "Recording to bag: %s" << bag_path.c_str());
    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}