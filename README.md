[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# udacity-robond-p2
Go Chase It! - Project 2 of the Udacity Robotics Software Engineer Nanodegree

## Instructions

1. Clone the repository

```git
git clone https://github.com/PenguinLemma/udacity-robond-p1.git PenguinLemmaRoboND_P2
```

2. Initialise catkin workspace and set env variables

```shell
cd PenguinLemmaRoboND_P2
catkin_make
source set_env_var_$VERSION.$SHELL
```
where $SHELL can be "sh", "bash" or "zsh" and $VERSION can be "pre_melodic" or "melodic".

For each new terminal we need to open, we will have to do
```shell
cd $PATH_TO_PARENT_DIR/PenguinLemmaRoboND_P2
source set_env_var_$VERSION.$SHELL
```

3. Launch the world
```
roslaunch my_robot world.launch
```
4. Launch ball_chaser nodes
```
roslaunch ball_chaser ball_chaser.launch
```

## Future improvements

- [ ] Add namespaces
- [ ] Extract constants
- [ ] Resize robot (chasis and sensors base radius, mainly) so that it fits through the doors ^^
- [ ] In chase_ball.cpp, in the case the ball was not found in the image, scan surroundings (set angular_z velocity to positive). After a full turn, if it's still not found, move randomly and remember to not scan surroundings for a while, to avoid ballroom dancing.
- [ ] Extract "scan_surroundings" and "go_somewhere_else" as actions

## License
Original version of the license of this repository can be found here:
https://gist.github.com/laramartin/7796d730bba8cf689f628d9b011e91d8
