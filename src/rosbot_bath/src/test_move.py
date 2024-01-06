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
    STOP = "Stop"
    
    

    # const thresholds and values
    SAFE_DISTANCE = 0.65 # lidar
    SAFE_DIST_RNG = 0.4 # IR
    YAW_THRESHOLD = 0.1
    FRWD_DIST_THRESHOLD = 0.9 # travelled forward distance threshold
    MAX_ANGULAR_VELOCITY = 1
    STEP = 2

    def __init__(self, namespace=""):
        
        # topic names
        self.TOPIC_IMU = "/" + namespace + "/imu"
        self.TOPIC_ODOM = "/" + namespace + "/odom"
        self.TOPIC_SCAN = "/" + namespace + "/scan"
        self.TOPIC_RANGEFL = "/" + namespace + "/range/fl"
        self.TOPIC_RANGEFR = "/" + namespace + "/range/fr"
        self.CMD_VEL_TOPIC = "/" + namespace + "/cmd_vel"
        
        # States
        self.current_state = "Forward"
        self.last_state = None

        # Global variables
        self.last_print_time = None
        self.odom = None
        self.total_distance = 0.0
        self.rev_distance = 0
        self.target_yaw = 0
        self.current_yaw = None
        self.pitch_counter = 0
        self.distance_in_fwd = 0
        self.yaw_error = 0

        # Range sensors
        self.range_fl = 0.7 
        self.range_fr = 0.7
        
        # Global variable to track the last turn time
        self.last_turn_time = 0
        self.turn_delay = 1.0  # Delay 

        # ros init
        rospy.init_node("obstacle_avoidance_fsm")
        
        #publisher
        self.pub = rospy.Publisher(self.CMD_VEL_TOPIC, Twist, queue_size=1)
        
        # subscribers
        rospy.Subscriber(self.TOPIC_RANGEFL, Range, self.Rangefl)
        rospy.Subscriber(self.TOPIC_RANGEFR, Range, self.Rangefr)
        rospy.Subscriber(self.TOPIC_SCAN, LaserScan, self.clbk_laser)
        rospy.Subscriber(self.TOPIC_ODOM, Odometry, self.odom_clbk)
        rospy.Subscriber(self.TOPIC_IMU, Imu, self.imu_clbk)

       

        
    


    # Range sensors clbks
    def Rangefl(self, msg):
        self.range_fl = msg.range

    def Rangefr(self, msg):
        self.range_fr = msg.range


    def clbk_laser(self, msg):
        
        lfront = slice(0*self.STEP,18*self.STEP)
        rfront = slice(342*self.STEP,360*self.STEP)
        right = slice(252*self.STEP, 288*self.STEP)
        left = slice(72*self.STEP, 108*self.STEP)

        regions = {
            "front": min(min(msg.ranges[lfront] + msg.ranges[rfront]), 10),
            "right": min(min(msg.ranges[right]), 10),
            "left": min(min(msg.ranges[left]), 10),
        }

        # State transition logic based on LiDAR data
          # Safe distance threshold for laser
        if regions["front"] < self.SAFE_DISTANCE:
        #if regions['front'] < safe_distance or range_fl < self.SAFE_DIST_RNG or range_fr < self.SAFE_DIST_RNG :

            if regions["left"] < regions["right"]:
                self.current_state = self.AVOID_OBSTACLE_RIGHT
            else:
                self.current_state = self.AVOID_OBSTACLE_LEFT
        
        elif self.distance_in_fwd >= self.FRWD_DIST_THRESHOLD :
                yaw_error = self.normalize_angle(0 - self.current_yaw)
                
                if abs(yaw_error) > self.YAW_THRESHOLD:
                    self.current_state = self.CORRECTION
                else:
                    self.current_state = self.FORWARD
        
        else:
            self.current_state = self.FORWARD
        
        #rospy.loginfo(f"Distance in clbk: {self.distance_in_fwd}")
    
        self.fsm_action()

    '''def clbk_laser(self, msg):
    # Existing region slices
        lfront = slice(0*self.STEP, 18*self.STEP)
        rfront = slice(342*self.STEP, 360*self.STEP)
        right = slice(252*self.STEP, 288*self.STEP)
        left = slice(72*self.STEP, 108*self.STEP)

        # New slices for front-left and front-right
        front_left = slice(18*self.STEP, 72*self.STEP)
        front_right = slice(288*self.STEP, 342*self.STEP)

        # Compute minimum distances for all regions
        regions = {
            "front": min(min(msg.ranges[lfront] + msg.ranges[rfront]), 10),
            "right": min(min(msg.ranges[right]), 10),
            "left": min(min(msg.ranges[left]), 10),
            "front_left": min(min(msg.ranges[front_left]), 10),
            "front_right": min(min(msg.ranges[front_right]), 10),
        }

        # State transition logic based on LiDAR data
        if regions["front"] < self.SAFE_DISTANCE:
            # Enhanced decision making considering front-left and front-right
            if regions["front_left"] < self.SAFE_DISTANCE and regions["front_right"] < self.SAFE_DISTANCE:
                # Both front-left and front-right are blocked
                self.current_state = self.REVERSE_OR_STOP
            elif regions["front_left"] < self.SAFE_DISTANCE:
                # Front-left is blocked
                self.current_state = self.AVOID_OBSTACLE_RIGHT
            elif regions["front_right"] < self.SAFE_DISTANCE:
                # Front-right is blocked
                self.current_state = self.AVOID_OBSTACLE_LEFT
            else:
                # Default avoidance based on left and right regions only
                if regions["left"] < regions["right"]:
                    self.current_state = self.AVOID_OBSTACLE_RIGHT
                else:
                    self.current_state = self.AVOID_OBSTACLE_LEFT
        elif self.distance_in_fwd >= self.FRWD_DIST_THRESHOLD:
            yaw_error = self.normalize_angle(0 - self.current_yaw)
            if abs(yaw_error) > self.YAW_THRESHOLD:
                self.current_state = self.CORRECTION
            else:
                self.current_state = self.FORWARD
        else:
            self.current_state = self.FORWARD

        # Call the state machine action
        self.fsm_action()'''
                       
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
        pitch = euler[1]
        self.current_yaw = euler[2]
        rospy.loginfo(f"Pitch: {pitch}\n")
        
        '''if abs(pitch) > 0.5:
            self.pitch_counter +=1
            if self.pitch_counter > 3:
                self.current_state = self.STOP
                self.pitch_counter = 0
                self.fsm_action()'''

        #rospy.loginfo(f"Target direction: {self.target_yaw}\nCurrent direction: {self.current_yaw}\n")


    def fsm_action(self):
        current_time = time.time()

        if self.current_state == self.FORWARD:
            if self.last_state != self.FORWARD:
                #rospy.loginfo("New FORWARD state detected. Resetting distance.")
                self.prev_distance = self.total_distance
            self.move_forward()
            self.distance_in_fwd = self.total_distance-self.prev_distance

            #rospy.loginfo(f"FWD Distance travelled in FSM: {self.distance_in_fwd}")
            #rospy.loginfo(f"TOTAL Distance travelled in FSM: {self.total_distance}")

        elif self.current_state == self.CORRECTION:
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

            # Check if the robot is close enough to the target orientation
            if abs(yaw_error) <= 0.1:  # Threshold to exit correction state
                self.current_state = self.FORWARD
                
            else:
                # Proportional control for smoother turning
                Kp = 1.5  # Gain factor, adjust as needed
                angular_velocity = Kp * yaw_error

                # Limit the angular velocity to avoid overcorrection
                
                angular_velocity = max(min(angular_velocity, self.MAX_ANGULAR_VELOCITY), -self.MAX_ANGULAR_VELOCITY)

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
        elif self.current_state == self.STOP:
            self.stop()


        #rospy.loginfo(f"State: {self.current_state}\n")
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
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0  # Forward speed
        msg.angular.z = 0   # No rotation
        self.pub.publish(msg)
        rospy.sleep(0.5) 

def main():
    
    rosbot_fsm_1 = RosbotFSM()
    rosbot_fsm_2 = RosbotFSM(namespace="second_rosbot")
    rospy.spin()

if __name__ == '__main__':
    main()
