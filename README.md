
# Navigation of Limo Robot for the detection of potholes using Yolov8


## Authors

- [@gdtdavies](https://www.github.com/gdtdavies) 27421138@students.lincoln.ac.uk

## Summary of Solution

navigation: I used LaserScan to find the distances from the walls. I separated the scans into three sections (left, front, and right). Turn left if the value to the right is too small. Vice-versa for turning right. Go forwards if nothing is close in front, otherwise reverse.
detection: I used yolov8 to train a model on labelled screenshots from the robot's perspective. Then I tuned the hyperparameters using Yolo's inbuilt tune function.
map placement: To avoid duplicate detections, it merges detections that are close. It publises the poses to /object_location, therefore they can be viewed in rviz.


## Run Locally

Install ros2 and dependencies
instructions here -> https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup

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


