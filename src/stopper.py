#!/usr/bin/python

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations
from math import atan2



class Stopper(object):

    def __init__(self, forward_speed, angular_speed,end_point_x,end_point_y):
        self.forward_speed = forward_speed  
        self.angular_speed = angular_speed
        self.min_dist_from_obstacle = 1
        self.is_colliding = False
	self.going_to_point = True
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan",LaserScan, self.scan_callback, queue_size=1)
	self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.start_point = Point()
	self.current_point = Point()
	self.end_point = Point()
	self.start_point.x = 0
	self.start_point.y = 0
	self.end_point.y = 0
	self.current_point.x = 0
	self.current_point.y = 0	
	self.end_point.x =  end_point_x
	self.end_point.y =  end_point_y
	self.forward_laser = 0
	self.right_laser = 0
	self.left_laser = 0
	self.current_action = "Left"
	self.theta = 0
	self.get_start_point = True
	self.passed_obstacle = False
	self.counter = 0
	self.finished = False
	self.print_once = True
	self.stat = "Start"
	time.sleep(5)	
	
    #move the turtle forward while not colliding
    def start_moving(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.is_colliding:
	    if not self.finished:
                self.go_to_point()
                rate.sleep()
	    if is_close(self.current_point.x,self.end_point.x,0.3) and is_close(self.current_point.y,self.end_point.y,0.3):
		self.finished = True
	    	self.finish()
	#change state if colliding
	self.check_state()


    #move around the obstacle 
    def follow_the_wall(self):
        rate = rospy.Rate(10)
        while self.is_colliding:
	    if self.current_action == "Left":
		self.rotate()
	    elif self.current_action == "Right":
		self.counter+=1
		self.turn_around()
	    self.print_status("Following The Wall")
        self.check_state()

    #check the state of the bot, if colliding avoid the collusion
    #otherwise continue moving 	
    def check_state(self):
	if self.is_colliding:
	    self.follow_the_wall()
	else:
	    self.start_moving()

    #go to the given point
    def go_to_point(self):
       x_inc = self.end_point.x  - self.current_point.x 
       y_inc = self.end_point.y  - self.current_point.y 
       angle = atan2(y_inc, x_inc)
       if abs(angle - self.theta) > 0.5:
	   self.rotate()
	   time.sleep(1)
       else:
           self.move_forward()
	
       self.print_status("Going To Point")





    #scan the ranges if the minimum distance from the obstacles is reached avoid it.
    #otherwise continue moving forward.
    def scan_callback(self, scan_msg):
        if self.is_on_line(self.current_point.x,self.current_point.y,0.1) and self.counter>=50:
            self.passed_obstacle = True
            self.counter = 0 
            self.is_colliding = False
	self.forward_laser = min(min(scan_msg.ranges[214:428]), 10)
	forward_range = scan_msg.ranges[214:428]
        self.right_laser =  min(min(scan_msg.ranges[0:214]), 10)
	self.left_laser = min(min(scan_msg.ranges[428:640]), 10)
        for dist in forward_range:
            if not self.passed_obstacle:          
	        if dist < self.min_dist_from_obstacle:
	            if not self.is_colliding:
		        self.is_colliding = True
		        break
	if self.is_colliding:
		if self.forward_laser <= 1.5:
			self.current_action = "Left"
		elif self.left_laser > 1.5:
			self.current_action = "Right"		
    


    #callback for odometry
    def odom_callback(self, pos_msg):
	if self.get_start_point:
		time.sleep(5)
		self.start_point.x = pos_msg.pose.pose.position.x
		self.start_point.y = pos_msg.pose.pose.position.y
		self.get_start_point = False
        self.current_point.x  = pos_msg.pose.pose.position.x
	self.current_point.y  = pos_msg.pose.pose.position.y
	orient = pos_msg.pose.pose.orientation
	(roll, pitch, self.theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])


    #turn around the obstecale
    def turn_around(self):
	rospy.sleep(0.1)
	move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        move_msg.angular.z = -self.angular_speed
	self.command_pub.publish(move_msg)


    #rotate the turtle
    def rotate(self):
	rospy.sleep(0.1)
	move_msg = Twist()
        move_msg.linear.x = 0
	move_msg.angular.z = self.angular_speed
	self.command_pub.publish(move_msg)


  #move the turtle forward
    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        move_msg.linear.z = 0
        self.command_pub.publish(move_msg)
	
    #check if the bot is in the line
    def is_on_line(self,x,y,limit):
	x_dif = self.end_point.x  - self.start_point.x	
	y_dif = self.end_point.y -  self.start_point.y
	x_dif = x_dif if x_dif!= 0 else 0.01
	a = y_dif/x_dif
	b = self.end_point.y - a * self.end_point.x
	return is_close(y,a*x +b,limit)

    #stop the bot after we finish
    def finish(self):
	    move_msg = Twist()
	    move_msg.linear.x=0
	    move_msg.linear.z=0
	    self.print_status("Found Target")


   #print the bot status
    def print_status(self,stat):
	if stat == "Going To Point" and self.stat!="Going To Point":
	    print("")
	    print("Current Status : Going To Point")
	    print("")
	    self.stat = "Going To Point"
	elif stat == "Following The Wall"  and self.stat!="Following The Wall":
	    print("")
            print("Current Status : Following The Wall")
	    print("")
	    self.stat = "Following The Wall"
        elif stat == "Found Target" and self.stat!= "Found Target":
	    print("")
            print("Current Status : Found Target!")
	    print("")
	    self.stat = "Found Target"

	
	


  

#check if two points are closed
def is_close(v1,v2,limit):

    return abs(v1 - v2) <= limit


    
	

	
	
	




	
	






    

    


   


  







