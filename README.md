# ENPM 700 ROS2 Programming Assignment 1
## Overview
This assignment is an introduction to creating a package in ROS2. This package called "beginner_tutorials" contains two executables, "listener" and "talker". 

Talker creates a node called "minimal publisher" which publishes std_msgs::msg::String type messages to a topic called "topic". This message is "Fibonacci sequence: x" where x is replaced with subsequent numbers from the fibonacci sequence. When the number becomes large enough to risk overflow with the long long datatype, it restarts from zero. 

Listener creates a node called "minimal subscriber" which subscribes to the "topic" topic and logs the messages it recieves, which display in the terminal as Info-level log messages. 

To create this package, I edited the following files from the tutorials:

- publisher_member_function.cpp
    - Replaced count_ variable with fib_a_ and fib_b
    - Created next_fib function to return current fibonacci number and calculate next one
    - Edited message sent in the timer_callback function
- CMakeLists.txt
    - Added find_package for rclcpp and stdmsgs
    - Added add_executable for listener and talker
    - Added install for listener and talker
- package.xml
    - Updated description, maintainer name and license
    - Added dependencies for rclcpp and stdmsgs
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
4. Run listener executable
```bash
ros2 run beginner_tutorials listener
```
5. Open new terminal and navigate to my_beginner_tutorials workspace
6. Run talker executable
```bash
source install/setup.bash
ros2 run beginner_tutorials talker
```





