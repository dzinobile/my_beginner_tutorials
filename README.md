# ENPM 700 ROS2 Programming Assignment 3
## Overview
This assignment builds on the previous assignment by modifying the package for added functionality.

The talker node now broadcasts a static tf frame called /talk with parent /world with non-zero translation and rotation.

A level 2 integration test was created to test the talker node. This test sends integers 7 and 3 to the FindDifference service and confirms the service returns -4. 

A new bag recorder node was created by modifying the ros tutorials SimpleBagRecorder example. This node records messages from all 4 topics that are active when the talker node is running:
- /chatter
- /parameter_events
- /rosout
- /tf
results are stored in a directory called results/my_bagx, where x is incremented to prevent overwriting previous my_bag directories.

A new launch file launches both the talker node and the bag recorder node. This launch file takes a boolean launch argument flag for "record". If record is set to "false", then the bag recorder node does not launch. 

## Assumptions
- Using ROS2 Humble
## Dependencies
### Standard Library
- chrono
- functional
- limits
- memory
- string
- filesystem
### From ROS2
- rclcpp
- std_msgs
- rcl_interfaces
- tf2_msgs
- tf2_ros
- rosbag2_cpp
- std_srvs
## Testing
- catch_ros2
### Python
- launch
- launch_ros
## Build / Run steps
### TF Frame Broadcast
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
4. Run the talker node
```bash
ros2 run beginner_tutorials talker
```
5. In a new terminal, verify the TF frames
```bash
source install/setup.bash
ros2 run tf2_ros tf2_echo world talk
```
6. In a new terminal, use view_frames to create a pdf
```bash 
ros2 run tf2_tools view_frames
```
### Integration Test
1. End all running programs and close all but 1 terminal
2. Run test from command line
```bash
colcon test --ctest-args tests integration_test_node
```
3. Examine test cases
```bash
colcon test-result --all --verbose
```
### Bag Recorder
1. Launch talker and bag recorder
```bash
ros2 launch beginner_tutorials bag_recorder.launch.py record:=true
```
2. Wait 15 seconds, then press ctrl+c to end program
3. Run listener node
```bash
ros2 run beginner_tutorials listener
```
4. In a new terminal, play back bag recording
```bash
source install/setup.bash
ros2 bag play results/my_bag1/
```
5. Verify listener begins printing messages from bag
6. Once bag play ends, verify record flag 


