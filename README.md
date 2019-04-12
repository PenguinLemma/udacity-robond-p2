# udacity-robond-p2
Go Chase It! - Project 2 of the Udacity Robotics Software Engineer Nanodegree

## Instructions

1. Clone the repository

```git
git clone https://github.com/PenguinLemma/udacity-robond-p1.git PenguinLemmaRoboND_P2
```

2. Build the catkin ws

```shell
cd PenguinLemmaRoboND_P2
catkin_make
```

3. Set the needed environment variables
```shell
source devel/setup.$SHELL
```
where $SHELL can be "sh", "bash" or "zsh".

In case of using ROS Melodic, the following command is also necessary:
```shell
export LC_NUMERIC="en_US.UTF-8"
```
This is due to: https://github.com/ros-visualization/rviz/issues/1249

4. Launch the world
```
roslaunch my_robot world.launch
```



## License
Original version of the license of this repository can be found here:
https://gist.github.com/laramartin/7796d730bba8cf689f628d9b011e91d8
