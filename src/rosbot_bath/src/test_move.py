#! /usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Global variables
current_state = "Forward"
last_print_time = None
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# State definitions
FORWARD = "Forward"
AVOID_OBSTACLE_LEFT = "AvoidLeft"
AVOID_OBSTACLE_RIGHT = "AvoidRight"

def clbk_laser(msg):
    global current_state, last_print_time

    if last_print_time is None:
        last_print_time = rospy.Time.now()
    
    step = 1
    
    # TODO FINISH THIS********************
    '''lfront = slice(0*step,18*step)
    rfront = slice(0*step,360*step)
    right = slice(252*step, 288*step)'''
# ****************************************

    regions = {
        'front': min(min(msg.ranges[:36] + msg.ranges[684:]), 10),
        'right': min(min(msg.ranges[504:576]), 10),
        'left': min(min(msg.ranges[144:216]), 10),
    }

    # State transition logic based on LiDAR data
    safe_distance = 0.75  # Example safe distance threshold
    if regions['front'] < safe_distance:
        if regions['left'] < regions['right']:
            current_state = AVOID_OBSTACLE_RIGHT
        else:
            current_state = AVOID_OBSTACLE_LEFT
    else:
        current_state = FORWARD

    # Call state action 
    fsm_action()

def odom_clbk(msg):x
    pass

# Global variable to track the last turn time
last_turn_time = 0
turn_delay = 1.0  # Delay 

def fsm_action():
    global current_state, last_turn_time
    current_time = time.time()

    if current_state == FORWARD:
        move_forward()
    elif current_state == AVOID_OBSTACLE_LEFT and (current_time - last_turn_time) > turn_delay:
        turn_left()
        last_turn_time = current_time
    elif current_state == AVOID_OBSTACLE_RIGHT and (current_time - last_turn_time) > turn_delay:
        turn_right()
        last_turn_time = current_time


def turn_left():
    msg = Twist()
    msg.linear.x = -0.1  # Slight backward movement
    msg.angular.z = 1.0  # Left turn
    pub.publish(msg)
    rospy.sleep(0.5)  # Back up for a short duration before turning

def turn_right():
    msg = Twist()
    msg.linear.x = -0.1  # Slight backward movement
    msg.angular.z = -1.0  # Right turn
    pub.publish(msg)
    rospy.sleep(0.5) 


def move_forward():
    msg = Twist()
    msg.linear.x = 0.5  # Forward speed
    msg.angular.z = 0   # No rotation
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('obstacle_avoidance_fsm')

    rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
