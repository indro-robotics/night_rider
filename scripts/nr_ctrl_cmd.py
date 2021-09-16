#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- nr_ctrl_cmd.py -----------------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that Subs to "/night_rider/CtrlCmd" and converts them to the 
#      serial MCU packages that is required by the MCU that controlls the motor driver
#---------------------------------------------------------------------------------------------------

import os
import struct
import time
from std_msgs.msg import UInt8, Int16
from geometry_msgs.msg import Twist
from night_rider.msg import CtrlCmd, RecvStatus
import rospy
import rospkg
import message_filters

debug = 0

class nr_ctrl_cmd: 
#=======================================================================
    def cmd_vel_callback(self, ros_cmd_vel) :
        if debug : rospy.loginfo(" ROS CALL BACK RECEIVED ")
        self.cmd_vel_from_ros = ros_cmd_vel
#-----------------------------------------------------------------------      
        
        
    def cmd_vel_to_nr_driver(self) :
        
        cmd_vel = self.cmd_vel_from_ros
        #print("----recevied angular.z: ", cmd_vel.angular.z)
        
        if debug :
            rospy.loginfo(" ROS cmd_vel data convertion")
            
        
        # ---- NR controller mode ------
        cmd_mode_ = 1  		# set mode to 1 = Auto/computer control (O = manual mode for the black controller)
        
        # ---- NR eStop state ------
        cmd_e_stop_ = 0	    # set to zero at all time by the computer estop is seperate system
        
        # ---- control of NR gears ------
        # To more forward mode need to be "drive"
        # To more backwards mode need to be "reverse"
        # safe state is "neutral" where NR will roll if push or under inertia
        # "cmd_vel.linear.x" is being used for both speed and direction to stay in ROS norms
        if cmd_vel.linear.x > 0.01 :
            cmd_gear_ = 0 				# Drive
        elif cmd_vel.linear.x < -0.01 :
            cmd_gear_ = 2				# Reverse
        else :
            cmd_gear_ = 1				# Neutral
            
		# ---- NR speed  ------
		# The NR drive use gears to define forward or backwards
		# The maxium unit to the NR MCU is 1000 units
		# Maxium speed of current geared NR is 8.2M/s (~30KM)
		# "scaler_nr_speed" is to match the MCU units to ROS twist m/s velosicty 
        
        cmd_speed_ = 0
        if cmd_gear_ == 0 :
            cmd_speed_ = int(cmd_vel.linear.x * self.scaler_nr_speed)
        elif cmd_gear_ == 2 :
            cmd_speed_ = int(cmd_vel.linear.x * -self.scaler_nr_speed)
        #print(cmd_speed_)
        # ---- NR steer  ------
        # As NR is Ackermen, cmd_velangular.z this normalized to 1.0 - 0 for maxium & minium steering (+/-28 deg)
        if cmd_vel.angular.z > 1.0 : cmd_vel.angular.z = 1.0
        if cmd_vel.angular.z <-1.0 : cmd_vel.angular.z = -1.0
        cmd_steer_ = int(cmd_vel.angular.z * 2000)
        
        # ---- NR steer  ------
        # As NR is is using negative "cmd_vel.linear.x" for gear and speed
        # the "break" will be controlled on cmd_vel.linear.z
        # if cmd_vel.linear.x = 0 and cmd_vel.linear.z = 0 the NR will be in neutral and just roll to a stop.
        if cmd_vel.linear.z > 0.01 :
            if cmd_vel.linear.z > 1 : cmd_vel.linear.z = 1 		# cmd_vel.linear.z normilized to 1.0 - 0 for maxium & minium (100 = full break)
            cmd_brake_ = cmd_vel.linear.z * 100
        else :
            cmd_brake_ = 0
            
        
        # --- Tidy & check data ---
        if cmd_speed_ > 1000 : cmd_speed_ = 1000
        if cmd_speed_ < 0 : cmd_speed_ = 0
        if cmd_steer_ > 2000 : cmd_steer_ = 2000
        if cmd_steer_ < -2000: cmd_steer_ = -2000
        if cmd_brake_ > 100 : cmd_brake_ = 100
        if cmd_brake_ <= 0 : cmd_brake_ = 0
        
        #print("------- mode: ", cmd_mode, "  stop: ", cmd_e_stop, "  gear: ", cmd_gear, "  speed: ", cmd_speed, "  steer ", cmd_steer, "  brake: ", cmd_brake)

        packet = [cmd_mode_, cmd_e_stop_, cmd_gear_, cmd_speed_, cmd_steer_, cmd_brake_]
        
        return packet
#-----------------------------------------------------------------------

#=======================================================================
    def pub_cmd(self, serial_msg_hz, mode, e_stop, gear, speed, steer, brake, alive) :
        ctrl_msg = CtrlCmd() 
        
        ctrl_msg.serial_msg_hz = serial_msg_hz
        ctrl_msg.mode = mode
        ctrl_msg.e_stop = e_stop
        ctrl_msg.gear = gear
        ctrl_msg.speed = speed
        ctrl_msg.steer = steer
        ctrl_msg.brake = brake
        ctrl_msg.alive = alive
        
        self.nr_cmd_pub.publish(ctrl_msg)
#-----------------------------------------------------------------------      
  
#=======================================================================
    def conversion(self, mode, e_stop, gear) :
        if mode == 0: 
            mode = "Manual"
        else:
            mode = "Auto"

        if e_stop == 0:
            e_stop = "OFF"
        else:
            e_stop = "ON"

        if gear == 0:
            gear = "Drive"
        elif gear == 2:
            gear = "Rear"
        else:
            gear = "Neutral"

        return mode, e_stop, gear
#-----------------------------------------------------------------------
        
#=======================================================================
    def sub_recv_status(self,RecvStatus) :
        print("--------------------status(sub)--------------------")
        print("recv_status(mode) : {}".format(RecvStatus.mode))
        print("recv_status(e_stop) : {}".format(RecvStatus.e_stop))
        print("recv_status(gear) : {}".format(RecvStatus.gear))
        print("recv_status(speed) : {}".format(RecvStatus.speed))
        print("recv_status(steer) : {}".format(RecvStatus.steer))
        print("recv_status(brake) : {}".format(RecvStatus.brake))
        print("recv_status(enc) : {}".format(RecvStatus.enc))
        print("recv_status(batt) : {}".format(RecvStatus.batt))
        print("recv_status(alive) : {}".format(RecvStatus.alive))
        print("---------------------------------------------------\n")
        return
#-----------------------------------------------------------------------

#=======================================================================
    def main_loop(self, serial_msg_hz) :
        
        cmd_packet = [0 for i in range(6)]     
        
        #rate = rospy.Rate(serial_msg_hz)
        rate = rospy.Rate(500)
        t_offset = time.time()

        self.alive = 0
        while not rospy.is_shutdown(): 
           t = time.time() - t_offset
                
           cmd_packet = self.cmd_vel_to_nr_driver()
                
           self.pub_cmd(
                serial_msg_hz,
                cmd_packet[0],
                cmd_packet[1],
                cmd_packet[2],
                cmd_packet[3],
                cmd_packet[4],
                cmd_packet[5],
                self.alive)
           
           self.alive = self.alive + 1
           
           # print(" time now: ", t)
           if self.alive == 256:
               self.alive = 0
               # print("----------- Alive reset -- time now: ", t)
               
           rate.sleep()
#----------------------------------------------------------------------- 

#=======================================================================
    def __init__(self):
        
        rospy.init_node("nr_cmd", anonymous=True)
        
        self.serial_msg_hz = rospy.get_param('~serial_msg_hz', 50)
        self.scaler_nr_speed = rospy.get_param('~speed_scaler', 225)  	# 1000 / Max speed(5m/s)  
        
        self.cmd_vel_from_ros = Twist() 
        
        self.nr_cmd_pub = rospy.Publisher("/night_rider/ctrl_cmd", CtrlCmd, queue_size=1)
        self.nr_cmd_vel_sub = rospy.Subscriber("/night_rider/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.nr_status_sub = rospy.Subscriber("/night_rider/recv_status", RecvStatus, self.sub_recv_status)
        
        self.main_loop(self.serial_msg_hz)
#-----------------------------------------------------------------------    

if __name__ == "__main__":
     try:
         nr_ctrl_cmd()
     except rospy.ROSInterruptException:
         pass
 
