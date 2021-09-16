# night_rider

This is the ROS driver for thr indro "night rider" platform. This ROS driver has been develop to allow standard ROS communication via the "/cmd_vel" topic, and to also allow for control of the LED lights on the night rider

**Publish ROS commands for driving control**

publish a ROS "twist" message type on to the "/night_rider/cmd_vel" topic  .


**Publish ROS commands for LED control**

Publish to a night_rider "LedCmd" message type on to the "/night_rider/led_light_control" topic.
Please the "msg" folder in this ROS driver for the message type decription

or

control the LED via ROS service call on "/night_rider/set_xxxxxxx_led", use "rosservice list" in the command terminal to see the full list


**Subscribe to night rider feedback**

Subscribe to a night_rider "RecvStatus" message type on to the "/night_rider/recv_status" topic.
Please the "msg" folder in this ROS driver for the message type decription
