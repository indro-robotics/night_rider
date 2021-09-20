# night_rider

This is the ROS driver for thr indro "Night Rider" platform, created by Indro Robotics, Canada. This ROS driver has been develop to allow standard ROS functionallity, allowing user to communication and control the platform. There are thre main parts, conrtol of the drive, control of the LED lights on the night rider and the feedback from the onboard embedded system.

![Night_Rider-GitHub-Profile-Indro-Robotics](https://user-images.githubusercontent.com/90868537/134020323-e6225486-889c-4ba5-930f-550a34252ee1.jpg)
_Picture: Night Rider Platform - Copyright Indro Robotics_


## Publish ROS commands for driving control

To control the speed and direction of the night rider, user only needs to publish a ROS **twist** message type on to the ROS topic:
```
/night_rider/cmd_vel
```
Please see here for more details on twist message type: http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html

The Night Rider has this configuration for the values in the twist message
```
linear: 
  x: 0.0   <---- This value controls the forward and backward direction (depending on the sign +/-), set up in standard meters per second (m/s)
  y: 0.0
  z: 0.0   <---- This value controls the brake and is normalized. therefor 0 = off, 1 = full brake and 0.5 = half brake
angular: 
  x: 0.0
  y: 0.0
  z: 0.0   <---- This value controls the steering and is normalized. therefor 0 = straight, -1 = max left and +1 = max right
```

## Publish ROS commands for LED control

Publish to a night_rider **LedCmd** message type on to the ROS topic:
```
/night_rider/led_light_control
```
Please see the "msg" folder in this ROS driver for the message type decription

or

Control the LED via **ROS service** call on 
```
/night_rider/set_xxxxxxx_led
```
This is the full list of LED services:

![Software ROS LED Services](https://user-images.githubusercontent.com/90868537/134018245-66c1c2c3-65b8-42c6-909d-215eafbbe0b0.png)

To check the service are running, in the command terminal use the follwoing command to see the full list of active services
```
$ rosservice list
```


## Subscribe to night rider feedback

Subscribe to a night_rider **RecvStatus** message type on to the ROS topic:
```
/night_rider/recv_status
```
Please see the "msg" folder in this ROS driver for the message type decription


# Support
If you have any question or issues with the code, please contact support@indrorobotics.com or add a comment on to this repo

### THANK YOU FROM INDRO ROBOTICS 
