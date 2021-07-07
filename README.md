# robotiq_gripper
The repository for robotiq gripper drivers in RIVeR lab

## Airpick
The airpick driver is written in a standalone python script without dependence on ROS (only some loggin functions are used, will update later) and a ROS wrapper is provided.

Currently, all the low-level parameters for `grip` and `release` are pre-configured for now using the `vacuum_grip()` and `vacuum_release()` function.

If more advanced configuration is needed, use `_req_vacuum_grip_raw` and `_req_vacuum_release_raw` function provided in the `airpick_tcp_control.py`.

### Setup

To integrate this package in your ROS workspace, first download the repos by 
```
git clone https://github.com/RIVeR-Lab/robotiq_gripper.git
```
Build the package by `catkin build` and then run
```
rosrun robotiq_gripper airpick_node.py
```
to initiate the server and write another client to call the service.

The ROS wrapper provided a service `/airpick_control_service` which receives `AirpickSimpleControl` request. Currently, the two support request commands are `grip` and `release`. 

Now you should be able to call the service either by writing a ROS service client or using the command line tool `rosservice`
