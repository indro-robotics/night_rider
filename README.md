# night_rider

This is the ROS driver for thr indro "night rider" platform. This ROS driver has been develop to allow standard ROS communication via the "/cmd_vel" topic, and to also allow for control of the LED lights on the night rider

## Publish ROS commands for driving control

publish a ROS **twist** message type on to the ROS topic:
```
/night_rider/cmd_vel
```
Please see here for more details on twist message type: http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html

night rider has this configuration for the values in the twist message
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

## Publish ROS commands for LED control**

Publish to a night_rider **LedCmd** message type on to the ROS topic:
```
/night_rider/led_light_control
```
Please the "msg" folder in this ROS driver for the message type decription

or

control the LED via ROS service call on 
```
/night_rider/set_xxxxxxx_led
```
use
```
$ rosservice list
```
in the command terminal to see the full list


## Subscribe to night rider feedback

Subscribe to a night_rider **RecvStatus** message type on to the ROS topic:
```
/night_rider/recv_status
```
Please the "msg" folder in this ROS driver for the message type decription


# Support
If you have any question or issues with the code, please contact support@indrorobotics.com or add a comment on to this repo

### THANK YOU FROM INDRO ROBOTICS 
