# info
This repository acts as a hub for the rest of the projects hosted through the QUTAS repositories, including supporting examples and style guides.

Check the wiki for a host of information on a range of examples, subsystem comparisons, design considerations for smart UAS

## Software Stacks
### kinetic_sample_packages
A collection of sample C++ and python packages to use as a base for other ROS nodes

### plume_sampler
A system to perform automatic air sampling when it detects that it is within a smoke plume

### marker_localization
A few small ROS nodes identify ARUCO markers, determine a map of the environment, and determine a relative pose of a the camera
 
## UAS Software 
### robin
A mavlink/mavros compatible firmware for Naze32-type flight control units

### mavel
A position controller for multirotors in an arbitrary local frame

### breadcrumb
A navigation controller with waypoint, failsafe, and external input capabilities.

# Other Tools
### teleop_offboard_attitude
A teleoperation interface to control UAS

### rqt_mc_tuner
A rqt GUI to assist with tuning a multirotor flight controller

### uavtaq_emulator
A system emulator for a university unit on systems engineering

### transform_rebroadcaster
Convenient ROS node to change TransformStamped messages to a PoseStamped message
