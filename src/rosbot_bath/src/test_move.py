#! /usr/bin/env python3
import time
import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


class RosbotFSM:

    # State definitions
    FORWARD = "Forward"
    AVOID_OBSTACLE_LEFT = "AvoidLeft"
    AVOID_OBSTACLE_RIGHT = "AvoidRight"
    CORRECTION = "Correction"

    def __init__(self):
        # States
        self.current_state = "Forward"
        self.last_state = None

        # Global variables
        self.last_print_time = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = None
        self.total_distance = 0.0
        self.rev_distance = 0
        self.target_yaw = 0
        self.current_yaw = None
        self.distance_in_fwd = 0
        self.yaw_error = 0

        # Range sensors
        self.range_fl = 0.7 
        self.range_fr = 0.7
        
        # Global variable to track the last turn time
        self.last_turn_time = 0
        self.turn_delay = 1.0  # Delay 

        # ros init
        rospy.init_node('obstacle_avoidance_fsm')
        
        # subscribers
        rospy.Subscriber('/range/fl', Range, self.Rangefl)
        rospy.Subscriber('/range/fr', Range, self.Rangefr)
        rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        rospy.Subscriber('/odom', Odometry, self.odom_clbk)
        rospy.Subscriber('/imu', Imu, self.imu_clbk)
    


    # Range sensors clbks
    def Rangefl(self, msg):
        self.range_fl = msg.range

    def Rangefr(self, msg):
        self.range_fr = msg.range


    def clbk_laser(self, msg):
        #if last_print_time is None:
            #last_print_time = rospy.Time.now()
        
        step = 2
    
        lfront = slice(0*step,18*step)
        rfront = slice(342*step,360*step)
        right = slice(252*step, 288*step)
        left = slice(72*step, 108*step)


        regions = {
            'front': min(min(msg.ranges[lfront] + msg.ranges[rfront]), 10),
            'right': min(min(msg.ranges[right]), 10),
            'left': min(min(msg.ranges[left]), 10),
        }


        # State transition logic based on LiDAR data
        safe_distance = 0.5  # Safe distance threshold for laser
        self.safe_dist_rng = 0.3
        if regions['front'] < safe_distance:
        #if regions['front'] < safe_distance or range_fl < safe_dist_rng or range_fr < safe_dist_rng :

            if regions['left'] < regions['right']:
                self.current_state = self.AVOID_OBSTACLE_RIGHT
            else:
                self.current_state = self.AVOID_OBSTACLE_LEFT
        
        elif self.distance_in_fwd >= 0.9:
                yaw_error = self.normalize_angle(0 - self.current_yaw)
                
                if abs(yaw_error) > 0.1:
                    self.current_state = self.CORRECTION
                else:
                    self.current_state = self.FORWARD
        
        else:
            self.current_state = self.FORWARD
        
        rospy.loginfo(f"Distance in clbk: {self.distance_in_fwd}")
    
        self.fsm_action()                       

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    

    def odom_clbk(self, msg):
        if self.odom is not None:
            dx = msg.pose.pose.position.x - self.odom.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.odom.pose.pose.position.y

            distance = np.sqrt(dx**2+dy**2)

            self.total_distance += distance

        self.odom = msg

        #rospy.loginfo("Total Traveled Distance: {:.2f} meters".format(total_distance))


    def imu_clbk(self, msg):

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
            )

        # convert to Euler 
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        self.current_yaw = euler[2]

        #rospy.loginfo(f"Target direction: {target_yaw}\nCurrent direction: {current_yaw}\n")


    def fsm_action(self):
        current_time = time.time()

        if self.current_state == self.FORWARD:
            if self.last_state != self.FORWARD:
                #rospy.loginfo("New FORWARD state detected. Resetting distance.")
                self.prev_distance = self.total_distance
            self.move_forward()
            self.distance_in_fwd = self.total_distance-self.prev_distance

            rospy.loginfo(f"FWD Distance travelled in FSM: {self.distance_in_fwd}")
            rospy.loginfo(f"TOTAL Distance travelled in FSM: {self.total_distance}")

        elif self.current_state == self.CORRECTION:
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

            # Check if the robot is close enough to the target orientation
            if abs(yaw_error) <= 0.1:  # Threshold to exit correction state
                self.current_state = self.FORWARD
                '''if yaw_error > 0:
                        # Turn right
                    msg = Twist()
                    msg.angular.z = -1 
                    pub.publish(msg)
                else:
                        # Turn left
                    msg = Twist()
                    msg.angular.z = 1  
                    pub.publish(msg)'''
                
            else:
                # Proportional control for smoother turning
                Kp = 1.5  # Gain factor, adjust as needed
                angular_velocity = Kp * yaw_error

                # Limit the angular velocity to avoid overcorrection
                max_angular_velocity = 1
                angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

                # Send turn command
                msg = Twist()
                msg.angular.z = angular_velocity
                self.pub.publish(msg)

        elif self.current_state == self.AVOID_OBSTACLE_LEFT and (current_time - self.last_turn_time) > self.turn_delay:
            self.turn_left()
            self.last_turn_time = current_time
        elif self.current_state == self.AVOID_OBSTACLE_RIGHT and (current_time - self.last_turn_time) > self.turn_delay:
            self.turn_right()
            self.last_turn_time = current_time
        
        rospy.loginfo(f"State: {self.current_state}\n")
        self.last_state = self.current_state


    def turn_left(self):
        msg = Twist()
        msg.linear.x = -0.15  # Slight backward movement
        msg.angular.z = 1.0  # Left turn
        self.pub.publish(msg)
        rospy.sleep(0.5)  # Back up for a short duration before turning

    def turn_right(self):
        msg = Twist()
        msg.linear.x = -0.15  # Slight backward movement
        msg.angular.z = -1.0  # Right turn
        self.pub.publish(msg)
        rospy.sleep(0.5) 


    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward speed
        msg.angular.z = 0   # No rotation
        self.pub.publish(msg)

def main():
    
    rosbot_fsm = RosbotFSM()
    rospy.spin()

if __name__ == '__main__':
    main()
