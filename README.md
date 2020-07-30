# sample cruise control

Repository for implementing sample cruise control as described in its corresponding design document and README file:

## Pre-requistes
foxy docker container should be available to execute this program

## Build from source

It is recommended to create a new overlay workspace on top of your current ROS 2 installation.

```
$ mkdir -p ~/WORKSPACE
$ cd ~/WORKSPACE
```

Clone this repository into the workspace:


Then build the rudimentary cruise control packages with this command:

```
$ source /opt/ros/foxy/setup.bash
$ colcon build --symlink-install --packages-select cruise_control
$ source install/setup.bash
```

#### Executing the programe

The tests can be run using the following commands:

```
Terminal 1:
$ ros2 launch cruise_control cruise_control.launch.py

Above command will start the cruise control application starts with velocity 10km/h
and after some time it will maintain the constant velocity 80km/h


Terminal 2:
$ ros2 run cruise_control userinput
```
The above program will allow the user to change the cruise control velocity to some different velocity
