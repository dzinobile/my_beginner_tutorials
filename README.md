# ENPM 700 ROS2 Programming Assignment 2
## Overview
This assignment builds on the previous assignment by modifying the package for added functionality.

Talker now contains a service called "FindDifference" that finds the difference between two input intergers. It also logs messages at 5 different logging levels. Whenever it publishes a message, it logs its activit at the INFO log level. It also logs a message at another log level that depends on the size of the current fibonacci value: 
- A DEBUG message is logged when the value is under 1,000,000
- A WARN message is logged when the value is between 1,000,000 and 1,000,000,000,000,000
- An ERROR message warning that the number is approaching the overflow limit is logged when the value is over 1,000,000,000,000,000
- A FATAL message is logged when the overflow limit is reached and the counter sets back to 0
These messages are all logged using the _STREAM API.

Listener was also updated to log its messages to the _STREAM API.

Both nodes can now be launched simultaneously using the newly created launch file.

## Assumptions
- Using ROS2 Humble
## Dependencies
### Standard Library
- chrono
- functional
- limits
- memory
- string
### From ROS2
- rclcpp
- stdmsgs
## Build / Run steps
1. Clone git repository
```bash
git clone https://github.com/dzinobile/my_beginner_tutorials.git && cd my_beginner_tutorials
```
2. Source ROS2, colcon-argcomplete, and colcon_cd
```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
```
3. Build package and source workspace
```bash
colcon build
source install/setup.bash
```
4. Launch talker and listener nodes
```bash
ros2 launch beginner_tutorials talker_listener.launch.py 
```
5. Call service to find difference between numbers
```bash
ros2 service call /subtract_two_ints beginner_tutorials/srv/FindDifference "{a: 3, b: 10}"
```






