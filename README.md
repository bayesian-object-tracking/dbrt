# ROS Depth-Based Robot Tracking Package (dbrt)

This package extends the object tracking packages dbot and dbot_ros to track 
articulated rigid bodies with several degree of freedom. In addition to depth
images, the robot tracker incorporates joint angle measurements at a higher 
rate, typically 100Hz-1kHz. Here are some of the core features

 * Provides joint state estimates at the rate of joint encoders
 * Compensates for inaccurate kinematics by estimating biases on the joint 
   angles
 * Estimates the head camera to robot base, if needed. Typically, the exact 
   camera location is unknown
 * Handles occlusion
 * Copes with camera delays 
 * Requires only the model, i.e. the URDF description including the link meshes.

## Getting Started Example

First of all, set up and run the example, as described in the [Getting Started](https://github.com/bayesian-object-tracking/getting_started#robot-tracking)
documentation.

## Setting Up Your Own Robot

Now you can use the working example as a starting point. To use your own robot, you will need
its URDF, and you will need to modify some launch and config files in [dbrt_example](https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started/tree/master/dbrt_example). The launch files
should be self explanatory and easy to adapt. You will need to edit 
the file fusion_tracker_gpu.launch (fusion_tracker_cpu.launch) to use
your own robot model, instead of Apollo. 

The main work will be to adapt the fusion_tracker_gpu.yaml 
(fusion_tracker_cpu.yaml) file to your robot. All the parameters 
for the tracking algorithm are specified in this file, and it is robot
specific. You will have to adapt the link and joint names to your robot.
Furthermore, you can specify which joints should be corrected using the 
depth images, how aggressively they should be corrected, and whether
you want to estimate an offset between the true camera and the 
nominal camera in your robot model. 

### URDF Camera Frame

Our algorithm assumes that the frame of the depth image (specified by
the camera_info topic) exists in your URDF robot model. You can check the camera frame
by running 
```bash
rostopic echo /camera/depth/camera_info.
```
If this frame does not exist in your robot URDF, you have to add such a camera frame to the 
part of the robot where the camera is mounted. This requires 
connecting a camera link through a joint to another link of the robot. Take a 
look at [head.urdf.xacro](https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started/blob/master/apollo_robot_model/models/head.urdf.xacro#L319) .

The XTION camera link *XTION_RGB* is connected to the link *B_HEAD* through the 
joint *XTION_JOINT*. The transformation between the camera and the robot is not 
required to be very precise, since our algorithm can estimate an offset. 
However, it must be accurate enough to provide 
a rough initial pose.



## How to cite?
```
@article{GarciaCifuentes.RAL,
 title = {Probabilistic Articulated Real-Time Tracking for Robot Manipulation},
 author = {Garcia Cifuentes, Cristina and Issac, Jan and W{\"u}thrich, Manuel and Schaal, Stefan and Bohg, Jeannette},
 journal = {IEEE Robotics and Automation Letters (RA-L)},
 volume = {2},
 number = {2},
 pages = {577-584},
 month = apr,
 year = {2017},
 month_numeric = {4}
}
```
