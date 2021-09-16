#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- hri_controller_remap.py -----------------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that subs to the published topics on "/joy" and 
#       "/hrI_src/emergency_stop" that are controller by the HRI (FORT Robotics) Wireless
#       controller (SRC)
#       This ros node has been configured for the indro robotics "night rider" platform,
#       to allow control of the leds via ros service calls and the "/cmd_val" topic
#       via publishing new "/cmd_val" vales, which is published to the night rider ros driver (nr_ctrl_cmd)
#---------------------------------------------------------------------------------------------------

import sys
import rospy
import rosservice
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from night_rider.msg import LedCmd

class hri_controller_to_nr: 
#=======================================================================
	def callbackJoy2twist(self, data):
#------ declearations
		twist = Twist()
		
		
		# ----- Braking -----
		# hri publishes on the "joy" topic where the left trigger 
		# is used as a "death man" / break safety input. Thispull down and 
		# keep this trigger down to dis-enage the break. If the use is not
		# focused or is kocked or worse, then this trigger will reset and the 
		# break will be enaged for safety means the user
		twist.linear.z = 1 - data.buttons[10]
		if twist.linear.z < 0.1 :
			self.led_cmd.rear_red_spot_led = 0
		else :
			self.led_cmd.rear_red_spot_led = 1
		
		
		# ----- Speed and Gear -----
		# hri publishes on the "joy" topic where the left analog stick
		# is use to deturmian the speed of the NR and
		# the direction of travel (forward or backward)
		# "joy" is normiziled to -1~0~1, this can be scale to the Maxium 
		# m/s of the NR, which is approx 5m/s with it curent gearing
		# Drive and reserves is deturmained by the postive/negitive of the joy value
		# if joy value = 0 the NR will be in neutarl and roll (if brake not engaged)  
		twist.linear.x = data.axes[1] * self.scaler_nr_joy2cmd_linear_x
		
		# ----- Steer -----
		# hri publishes on the "joy" topic where the right analog stick
		# is use to deturmian the steer of the NR and
		# the direction of travel via turning (right or left)
		# "joy" is normiziled to -1~0~1, for NR platform the maxium
		# angle of steering is 28 deg. (nr_ctrl_cmd.py has the scaler) 
		# Inverted "data.axes" as the pos and neg numbers are oppersite in ROS TF
		twist.angular.z = -data.axes[2] 
		
		# ----- Front LED Spots -----
		if data.buttons[6] == 1 and self.last_button_state[6] == False :
			self.led_cmd.front_spot_leds = not self.led_cmd.front_spot_leds
			self.last_button_state[6] = True 
		if data.buttons[6] == 0 :
			self.last_button_state[6] = False
			
		# ----- Rear LED Spots -----
		if data.buttons[4] == 1 and self.last_button_state[4] == False :
			self.led_cmd.rear_spot_leds = not self.led_cmd.rear_spot_leds
			self.last_button_state[4] = True 
		if data.buttons[4] == 0 :
			self.last_button_state[4] = False
		
		# ----- Left yellow LEDs  -----
		if data.buttons[7] == 1 and self.last_button_state[7] == False :
			self.led_cmd.rear_left_yellow_led = not self.led_cmd.rear_left_yellow_led
			self.led_cmd.front_left_yellow_led = not self.led_cmd.front_left_yellow_led
			self.last_button_state[7] = True 
		if data.buttons[7] == 0 :
			self.last_button_state[7] = False
		
		# ----- right yellow LEDs  -----
		if data.buttons[5] == 1 and self.last_button_state[5] == False :
			self.led_cmd.rear_right_yellow_led = not self.led_cmd.rear_right_yellow_led
			self.led_cmd.front_right_yellow_led = not self.led_cmd.front_right_yellow_led
			self.last_button_state[5] = True 
		if data.buttons[5] == 0 :
			self.last_button_state[5] = False
		
		# ----- publish to led_control topic -----
		self.pubLEDs.publish(self.led_cmd)
		
		# ----- publish to /cmd_vel  -----
		self.pubTwist.publish(twist)
#-----------------------------------------------------------------------  
		
#=======================================================================
	def main_loop(self) :
		rospy.loginfo("HRI Joy To Night Rider Node Started")

		while not rospy.is_shutdown(): 
			
			rospy.spin()
#-----------------------------------------------------------------------  
			
#=======================================================================
	def __init__(self):
		# publishing to "/cmd_vel" to control night rider
		global pubTwist, pubLEDs
		global led_cmd, led_cmd_pervious, last_button_state
		
		self.led_cmd = LedCmd()
		self.led_cmd_pervious = LedCmd()
		self.last_button_state = [False for i in range(11)]
		
		rospy.init_node('hri_controller_to_nr')
		
		self.pubTwist = rospy.Publisher('/night_rider/cmd_vel', Twist, queue_size=10)
		self.pubLEDs = rospy.Publisher('/night_rider/led_light_control', LedCmd, queue_size=1)
		
		# subscribed to joystick inputs on topic "joy"
		rospy.Subscriber("joy", Joy, self.callbackJoy2twist)
		
		self.scaler_nr_joy2cmd_linear_x = rospy.get_param('~joy2cmd_linear_x', 4)
		self.scaler_nr_joy2cmd_node_hz = rospy.get_param('~joy2cmd_node_hz', 20)
		rospy.Rate(self.scaler_nr_joy2cmd_node_hz)
		
		# starts the node
		self.main_loop()
#-----------------------------------------------------------------------  

if __name__ == '__main__':
    hri_controller_to_nr()
