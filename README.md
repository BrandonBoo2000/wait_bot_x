# Wait-Bot-X
Group Project for WID3005 Intelligent Robotic
## Wait-Bot-X Setup
Cloning the Wait-Bot-X to your workspace
```
cd [workspace]/src
git clone https://github.com/BrandonBoo2000/wait_bot_x.git
cd ..
catkin_make
```

Setup Turtlebot
Follow below link to setup turtlebot
```
https://github.com/gaunthan/Turtlebot2-On-Melodic
```

## Ordering System
This part is an interaction between Wait-Bot-X with customer
First, to initiate the ordering system, you should open terminal and run the *roscore*
```
roscore
```
Next, open another terminal to run below command
```
rosrun wait_bot_x ordering_system.py
```
There are several speech can be detected to perform respective task when ordering is started
- menu: display the menu
- repeat: repeat the order
- done: confirm the order with customer
- thank you: end the ordering system
- food ordering
  - must state the item and the amount of the items
  - example: I want a fried chicken, six burger and etc

Menu
```
menu = ["nasi lemak", "pizza", "burger", "fried chicken", "pepsi", "orange juice", "apple juice", "coffee"]
```

## Navigation System
Build the map and navigation for Wait-Bot-X

### SLAM Map Building
Make sure you run *roscore* in another terminal before begin with this part.
Run below command one by one in different terminal to generate the map for navigation later
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/[username]/[workspace]/src/wait_bot_x/maps/res_map.world
roslaunch turtlebot_gazebo gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
rosrun map_server map_saver -f /home/[username]/[workspace]/src/wait_bot_x/maps/restaurant_map
```

### Autonomous Navigation
Make sure you run *roscore* in the different terminal before begin with this part
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/[username]/[workspace]/src/wait_bot_x/maps/res_map.world
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/[username]/[workspace]/src/wait_bot_x/maps/restaurant_map.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch rchomeedu_navigation navigation.launch
```
