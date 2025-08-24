source ./install/setup.bash
sleep 10
ros2 launch turtlebot3_gazebo labyrinths.launch.py use_sim_time:=true&
sleep 10
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true