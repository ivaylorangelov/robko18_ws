install pylinter
sudo apt-get update
sudo apt-get install pylint

Shift + Alt + F format code VS code
source ~/robko18_ws/devel/setup.bash
source devel/setup.bash

echo $ROS_PACKAGE_PATH

sh run_gazebo.sh
sh run_rviz.sh
sh run_teleop.sh 

Teleop:
roslaunch robko18_navigation robko18_teleop.launch

BUILDING MAP
Creating the Map 
Run the following commands below. Use the teleop to move the robot around to create an accurate and thorough map.
Don't forget to source each terminal you open!!!!!!! source ~/robko18_ws/devel/setup.bash
In Terminal 1, launch the Gazebo world
roslaunch ~/robko18_ws/src/robko18_gazebo/robko18_world.launch
In Terminal 2, start map building
roslaunch ~/robko18_ws/src/robko18_navigation/gmapping_demo.launch
In Terminal 3, launch rviz and set the following parameters:
roslaunch ~/robko18_ws/src/robko18_description/robko18_rviz_gmapping.launch
In Terminal 4, start teleop
roslaunch ~/robko18_ws/src/robko18_navigation/robko18_teleop.launch
Saving the Map 
In Terminal 5, save the map to some file path
rosrun map_server map_saver -f ~/robko18_ws/src/robko18_navigation/maps/choose-a-name_map

USING test_map MAP for Navigation
Don't forget to source each terminal you open!!!!!!! source ~/robko18_ws/devel/setup.bash
terminal1:
roslaunch ~/robko18_ws/src/robko18_gazebo/robko18_world.launch or
roslaunch ~/robko18_ws/src/robko18_gazebo/robko18_willowgarage_world.launch
roslaunch ~/robko18_ws/src/robko18_gazebo/robko18_kitchen_dining_world.launch 
terminal2:
roslaunch ~/robko18_ws/src/robko18_navigation/amcl_demo.launch  or 
roslaunch ~/robko18_ws/src/robko18_navigation/amcl_willowgarage.launch
roslaunch ~/robko18_ws/src/robko18_navigation/amcl_kitchen_dining.launch
terminal3:
roslaunch ~/robko18_ws/src/robko18_description/robko18_rviz_amcl.launch

********Start simulated stansalone crpmover4 arm with ros controll*****************
Don't forget to source each terminal you open!!!!!!! source ~/robko18_ws/devel/setup.bash
launch:
~/robko18_ws/src/robko18_gazebo/launch/roslaunch cprmover4_ros_control_test.launch

You can move the arm in the simulation manually by publishing to cprmover4 joint controller command topics --- example:
rostopic list:
rostopic pub -1 /cprmover4/joint3_controller/command std_msgs/Float64 "data: 2"

********Start simulated stansalone crpmover4 arm with moveit and gazebo ros controll*****************
Don't forget to source each terminal you open!!!!!!! source ~/robko18_ws/devel/setup.bash
roslaunch ~/robko18_ws/src/robko18_gazebo/launch/cprmover4_sim_for_moveit_bringup.launch

wait for the model and text OMPL in green color to appear below Planning library on Rviz screen and move the robot or plan and execute trajectories
 
rosrun rqt_graph rqt_graph

So you can e.g. use grep to the output of rostopic list to get the list of available actions, e.g.
rostopic list | grep -o -P '^.*(?=/feedback)'

*******************************
show non ASCII symbols in file:
grep -P '[^\x20-\x7E]' robko18.xacro
find files in subdirs containing string 
grep -R "put_string_here" . | cut -d ":" -f 1

********************************
My Github skydancerbg no disclosure e-mail: 
13396312+skydancerbg@users.noreply.github.com

https://github.com/skydancerbg/robko18_ws.git
or push an existing repository from the command line. in the dir:
first set:
git config --global user.name "John Doe"
git config --global user.email johndoe@example.com

git init
git add . or git add --all
git commit -m 'First commit'
git remote -v
git remote add origin https://github.com/skydancerbg/robko18_ws.git
git push -u origin master
clone from: 
git@github.com:skydancerbg/robko18_ws.git
*********************
clone a branch from github:
git clone -b mybranch --single-branch git://sub.domain.com/repo.git
*********************
Zapomnia i ne pita za GithubParola za x sekundi (tuk 30 dni v sekundi)
git config --global credential.helper cache
git config --global credential.helper 'cache --timeout=2592000'
*********************
git@github.com:skydancerbg/robot_design.git   ---test git
******************************************
git clone -b kinetic --single-branch https://github.com/turtlebot/turtlebot.git
*************************************
Teleop package kinetic
http://wiki.ros.org/teleop_twist_keyboard
sudo apt-get install ros-kinetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
***************************************
EDIT bashrc:
nano ~/.bashrc
Once you've made any changes, you can get them to take effect in your current shell with:
source ~/.bashrc
****************
These are some of the pre-requisites of using directly the script name:
Add the sha-bang {#!/bin/bash) line at the very top.
Using chmod u+x scriptname make the script executable.
Place the script under /usr/local/bin folder.
Run the script using just the name of the script.
Other solution run like:
sh name_of_script.sh
*************************
*******install ros control******************
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control

install 
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-moveit-simple-controller-manager
sudo apt-get install ros-kinetic-joint-trajectory-controller
*************************

TURTLEBOT instalation !!!!!!!!!!! http://moorerobots.com/blog/post/3 !!!
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

*********************
So that I can use urdf_to_graphix to get a visual urdf?????
******************************
Give execute permission to your script:

chmod +x /path/to/yourscript.sh
And to run your script:

/path/to/yourscript.sh
Since . refers to the current directory: if yourscript.sh is in the current directory, you can simplify this to:

./yourscript.sh