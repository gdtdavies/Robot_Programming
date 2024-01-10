
# Navigation of Limo Robot for the detection of potholes using Yolov8

Description from the assignment 1 breifing CMP9767:
In this assignment, you will implement a software system for a mobile robot deployed to solve
a road inspection task. The goal of the task is to automatically appraise the quality of the road
surface and report that information to the human user. The basic report will simply contain the
number of detected potholes in the whole environment, whilst more advanced reports can
include the pothole size, their location and a pothole map indicating the severity of the road
damage across the environment.
The robotic platform for this task is a LIMO robot together with its sensors deployed in a
rectangular arena resembling an urban road scenario with a series of different “potholes”
placed at various locations. There will be two variants of the environment available: one
featuring easy to spot potholes with distinct appearance and one with more realistic
appearance for additional challenge. The robot should navigate autonomously in the
environment whilst undertaking the inspection task and aim to cover all the road segments.
The basic implementation will demonstrate the pothole counting functionality in an “easy
variant” of the simulation with the robot autonomously covering most of the environment and
starting from a fixed position.
The following features will count towards additional credit: the advanced pothole report,
flexible deployment from a random location or in a different layout of the environment, and the
physical demonstration of the functionality with the real robot. As a stretch assignment, the
robot will navigate over or around the potholes if that is physically possible.
You are encouraged to use any existing off-the-shelf ROS2 components and code fragments
and integrate them into your system, but you must correctly cite all the used resources in the
source code.
An excellent implementation solves the task accurately with innovative use of techniques not
covered in the module and is very well documented in the source code. It may also feature
meaningful unit and system tests to ensure high code quality. You will be using the Gazebo
simulator, providing you with simulated robots, and a world to operate in. In addition, you will
be able to demonstrate your software on a real robot and a real test scenario. The simulation
setup is presented in more detail in https://github.com/LCAS/CMP9767_LIMO/wiki/
Assignment-Setup.


## Authors

- [@gdtdavies](https://www.github.com/gdtdavies) 27421138@students.lincoln.ac.uk


## Run Locally

Clone the project

```bash
  git clone https://github.com/gdtdavies/Robot_Programming
  git clone https://github.com/LCAS/limo_ros2
  cd limo_ros2
  ./build.sh
  cd ../
```

```bash
  cd Robot_Programming
  pip install -r requirements.txt
  colcon build --symlink-install
```
Start up gazebo and rviz
```bash
    cd shell
    ./gazebo.sh #(or ./gazebo_simple.sh for the simple version)
    ./rviz2.sh
```
run the navigator
```bash
    ./nav.sh
```
run the publishers
```bash
    ./ground_truth
    ./get_transform
```
run the pothole detector
```bash
    ./detect.sh #(or ./map_simple.sh for the simple version)
```


