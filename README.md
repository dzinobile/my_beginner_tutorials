# ENPM 700 ROS2 Programming Assignment 2
## Overview
This assignment builds on the previous assignment by modifying the package for added functionality.

Talker now contains a service called "FindDifference" that finds the difference between two input intergers. It also logs messages at 5 different logging levels. Whenever it publishes a message, it logs its activit at the INFO log level. It also logs a message at another log level that depends on the size of the current fibonacci value: 
- A DEBUG_STREAM message is logged when the value is under 1,000,000
- A WARN_STREAM message is logged when the value is between 1,000,000 and 1,000,000,000,000,000
- An ERROR_STREAM message warning that the number is approaching the overflow limit is logged when the value is over 1,000,000,000,000,000
- A FATAL_STREAM message is logged when the overflow limit is reached and the counter sets back to 0
These messages are all logged using the _STREAM API.

Listener now outputs text in a color determined by the user. The user can put an input argument when running the launch file to change the color to red, green, yellow, blue, or white, with a default value of white.

Both nodes can now be launched simultaneously using the newly created launch file, which takes an input argument for the text color. The service added to the publisher can be accessed using the command line.

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
### Python
- launch
- launch_ros
## Build / Run steps
### Assignment 2 Deliverables
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
4. Launch talker and listener nodes(replace "red" with desired color: red, green, yellow, blue, or white)
```bash
ros2 launch beginner_tutorials talker_listener.launch.py listener_text_color:=red
```
5. From a new terminal, call service to find difference between numbers
```bash
source install/setup.bash
ros2 service call /subtract_two_ints beginner_tutorials/srv/FindDifference "{a: 3, b: 10}"
```
### Run Talker and Listener Separately
1. Complete steps 1-3 above if not already done:
```bash
git clone https://github.com/dzinobile/my_beginner_tutorials.git && cd my_beginner_tutorials
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
colcon build
source install/setup.bash
```
2. Run listener (replace "red" with desired color: red, green, yellow, blue, or white)
```bash
ros2 run beginner_tutorials listener --ros-args -p text_color:=red
```
3. From a new terminal, run talker at DEBUG log level
```bash
source install/setup.bash
ros2 run beginner_tutorials talker --ros-args --log-level DEBUG
```



