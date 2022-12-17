This repo is to create haptic interaction between multi-robot systems. It subscribes the robot states of the robots
that are listed under `param/robot_params.yaml` and publishes desired interaction torques based on the interaction
parameters by dynamic_reconfigure (can be accessed by rqt_gui).

# Build
```bash
$ catkin build multi_robot_interaction
```
# Launch
```bash
$ roslaunch multi_robot_interaction x2_dyad.launch
```

