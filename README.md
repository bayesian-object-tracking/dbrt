# ROS Depth Based Robot Tracking Library (dbrt)

This package provides three robot trackers
* Rotary tracker: A robot tracker based on joint angle measurements. This 
  tracker runs typically at 100Hz or 1kHz
* Visual tracker: This tracker uses depth images from a Kinect or XTION to 
  estimate the joint state. The filter is based on the Rao-Blackwellized 
  corrdinate descent particle filter implemented in 
  (dbot)[https://github.com/bayesian-object-tracking/dbot] package. The tracking 
  rate lies between 5Hz to 30Hz depending on the degree-of-freedom and model
  configurations. 
* Fusion tracker: This tracker fuses both filters mentioned above. The fusion 
  is performed while taking into considering the camera delay and the 
  computational time of the vision filter. 

## Running robot trackers with default configuration

These trackers are robot specific. You will need to create your own package and
add your config files and URDF models there. 
Take (dbrt_apollo)[https://git-amd.tuebingen.mpg.de/amd-clmc/dbrt_apollo.git] 
Apollo robot configuration package as an example or a template.

Make sure the robot is publishing the joint state to `/joint_states` topic. If 
you are using the visual or the fusion tracker, make also sure that the depth 
camera is providing depth images to a topic of choice spcified in the 
`dbrt_my_robot/config/camera.yaml`

Launching the trackers takes a single `roslaunch` call
```bash
roslaunch dbrt_apollo rotary_tracker.launch
```
```bash
roslaunch dbrt_apollo visual_tracker.launch
```
```bash
roslaunch dbrt_apollo fusion_tracker.launch
```

## Running robot trackers with custom configuration
The predefined launch files allow you to pass a custom config file
```bash
roslaunch dbrt_apollo rotary_tracker.launch rotary_tracker_config:=my_custom_rotary_tracker_config.yaml
```
```bash
roslaunch dbrt_apollo visual_tracker.launch visual_tracker_config:=my_custom_visual_tracker_config.yaml
```
```bash
roslaunch dbrt_apollo fusion_tracker.launch fusion_tracker_config:=my_custom_fusion_tracker_config.yaml
```

