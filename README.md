# ROS Depth-Based Robot Tracking Package (dbrt)

This package extends the object tracking packages, dbot and dbot_ros to track 
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

First follow the steps of setting up the object tracking as described in
Check out the [dbrt_getting_started](https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started.git)
for a full example of a robot setup with recorded data. You can find the setup 
steps in the (Getting Started)[https://github.com/bayesian-object-tracking/getting_started#robot-tracking]
documentation.

## Setting Up Your Own Robot
Provided a URDF, you only need adapt the Fusion Tracker config. For that take 
the [dbrt_example](https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started/tree/master/dbrt_example)
as an example. 

In the Fusion Tracker config file, you have map all the joint names to 
uncertainty standard deviations for the joint process model and joint 
observation models. The [dbrt_example](https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started/tree/master/dbrt_example) 
package provides a good starting point.

### URDF Camera Frame

In case your URDF model does not specify a camera link, you have to attach 
one to some part of the robot where the camera is mounted. This requires 
connecting a camera link through a joint to another link of the robot. Take a 
look at (head.urdf.xacro)[https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started/blob/master/apollo_robot_model/models/head.urdf.xacro#L319].
The XTION camera link *XTION_RGB* is linked to the link *B_HEAD* through the 
joint *XTION_JOINT*. The transformation between the camera and the robot is not 
required to be very precise. However, it must be accurate enough to provide 
a rough initial pose.

Finally, the camera link name (here XTION_RGB) must match the camera frame 
provided by the point cloud topic. To determine the name of the depth camera 
frame or the RGB frame if registration is used, run 

```bash
rostopic echo /camera/depth/camera_info
```

