cmake_minimum_required(VERSION 2.8.3)
project(sim_noise_thrusters)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  cola2_msgs

  # COLA2 Dependencies
  smarty200_description
)

# Declare things to be passed to dependent projects
catkin_package()

