#!/usr/bin/python
import rospy, sys
from stopper import Stopper

if __name__ == "__main__":
    rospy.init_node("stopper_node", argv=sys.argv)
    forward_speed = 0.5
    rotate_speed = 0.5
    if rospy.has_param('~forward_speed'):
        forward_speed = rospy.get_param('~forward_speed')
    if rospy.has_param('~angular_speed'):
        rotate_speed = rospy.get_param('~angular_speed')
    if rospy.has_param('~end_point_x'):
        end_point_x = rospy.get_param('~end_point_x')
    if rospy.has_param('~end_point_y'):
        end_point_y = rospy.get_param('~end_point_y')
    my_stopper = Stopper(forward_speed, rotate_speed,end_point_x,end_point_y)
    my_stopper.start_moving()
