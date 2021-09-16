#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- led_control.py --------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that connects to a serial port and pushes 
#       out a string of information define by the numato Lab 8 channel relay 
#       controller.
#       This ros node has been configured for the indro robotics "night rider" 
#       platform, to allow control of the leds via ros service calls
#------------------------------------------------------------------------------

import rospy
import serial
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import ByteMultiArray
from night_rider.msg import LedCmd


class led_control:
#-------------- function deleceration -----------------------------------------
#=======================================================================
    def control_relay_write_read(self, req, relay):
        if(req.data):
            cmd = str("on")
        else:
            cmd = str("off")
        self.serial_port.write("relay " + cmd + " " + str(relay) + "\n\r")
        self.serial_port.flush()
        self.serial_port.write("relay read " + str(relay) + "\n\r")

        response = self.serial_port.read(25)
        if ((response.find("on") > 0) and (req.data)):
            state = True
            response = "Turn on relay " + str(relay)

        elif ((response.find("off") > 0) and (req.data == False)):
            state = True
            response = "Turn off relay " + str(relay)
        else:
            state = False
            response = "!! ERROR !! not able to control relay"
        return SetBoolResponse(state, response)

#=======================================================================
    def control_relay_write(self, req, relay):
        if(req.data):
            cmd = str("on")
        else:
            cmd = str("off")
        self.serial_port.write("relay " + cmd + " " + str(relay) + "\n\r")
        self.serial_port.flush()
        return

#=======================================================================
    def control_relay_led_cmd(self, ledcmd):
        
        array = [ledcmd.rear_red_spot_led,
                 ledcmd.rear_spot_leds,
                 ledcmd.rear_left_yellow_led,
                 ledcmd.rear_right_yellow_led,
                 ledcmd.front_left_yellow_led,  
                 ledcmd.front_right_yellow_led, 
                 ledcmd.front_spot_leds,
                 ledcmd.spare
                 ]
        led = 0
        for x in array :
            if x :
                cmd = str("on")
            else :
                cmd = str("off")
            self.serial_port.write("relay " + cmd + " " + str(led) + "\n\r")
            self.serial_port.flush()
            led = led + 1
        
        return    

#=======================================================================
    def control_led_0(self, req):
        return self.control_relay_write(req, 0)  

    def control_led_1(self, req):
        return self.control_relay_write(req, 1)
    def control_led_2(self, req):
        return self.control_relay_write(req, 2)

    def control_led_3(self, req):
        return self.control_relay_write(req, 3)

    def control_led_4(self, req):
        return self.control_relay_write(req, 4)

    def control_led_5(self, req):
        return self.control_relay_write(req, 5)

    def control_led_6(self, req):
        return self.control_relay_write(req, 6)

    def control_led_7(self, req):
        return self.control_relay_write(req, 7)

#=======================================================================
    def control_all_leds(self, req):
        self.control_relay_write(req, 0)
        self.control_relay_write(req, 1)
        self.control_relay_write(req, 2)
        self.control_relay_write(req, 3)
        self.control_relay_write(req, 4)
        self.control_relay_write(req, 5)
        self.control_relay_write(req, 6)
        self.control_relay_write(req, 7)
        response = "All LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def control_right_yellow_leds(self, req):
        self.control_relay_write(req, 3)
        self.control_relay_write(req, 5)
        response = "Right Yellow LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def control_left_yellow_leds(self, req):
        self.control_relay_writ(req, 2)
        self.control_relay_write(req, 4)
        response = "Left Yellow LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def main_loop(self):
        rospy.loginfo("Night Rider LED Control Node Started")
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
        self.serial_port.close()

#-----------------------------------------------------------------------

#=======================================================================
    def __init__(self):
#-------------- Serial setup --------------------------------------------------
        self.port = rospy.get_param('~port', '/dev/ttyACM3')
        self.baud = rospy.get_param('~baud', 19200)
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
#-------------- function deleceration -----------------------------------------
        rospy.Service('/night_rider/set_rear_red_led', SetBool, self.control_led_0)
        rospy.Service('/night_rider/set_rear_spot_leds', SetBool, self.control_led_1)
        rospy.Service('/night_rider/set_rear_left_yellow_led', SetBool, self.control_led_2)
        rospy.Service('/night_rider/set_rear_right_yellow_led', SetBool, self.control_led_3)       
        rospy.Service('/night_rider/set_front_left_yellow_led', SetBool, self.control_led_4)
        rospy.Service('/night_rider/set_front_right_yellow_led', SetBool, self.control_led_5)
        rospy.Service('/night_rider/set_front_spot_leds', SetBool, self.control_led_6)
        rospy.Service('/night_rider/set_spare', SetBool, self.control_led_7)
        
        rospy.Service('/night_rider/set_all_leds', SetBool, self.control_all_leds)
        rospy.Service('/night_rider/set_right_yellow_leds', SetBool, self.control_right_yellow_leds)
        rospy.Service('/night_rider/set_left_yellow_leds', SetBool, self.control_left_yellow_leds)
        
        rospy.Subscriber("/night_rider/led_light_control", LedCmd, self.control_relay_led_cmd)
        
        global global_toggle
        global_toggle = False
        
        
        self.main_loop()
#-----------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('Led_control_server')
    nri = led_control()
