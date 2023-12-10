#include "rosbot_bath/rosbot_class.h"

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
// #include <opencv2/opencv.hpp>

ROSbot::ROSbot()
    : nh("~"),
      laser_topic("/scan"),
      vel_topic("/cmd_vel"),
      odom_topic("/odom") {
  vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName(vel_topic), 1);
  laser_sub = nh.subscribe(laser_topic, 10, &ROSbot::laser_callback, this);
  odom_sub = nh.subscribe(odom_topic, 10, &ROSbot::odom_callback, this);
  usleep(2000000);
}

void ROSbot::laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
  laser_range = laser_msg->ranges;
  // ROS_INFO("Laser value: %f", laser_range);
}
void ROSbot::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  pos_x = odom_msg->pose.pose.position.x;
  pos_y = odom_msg->pose.pose.position.y;
  pos_z = odom_msg->pose.pose.position.z;
  ROS_INFO_STREAM("Odometry: x=" << pos_x << " y=" << pos_y << " z=" << pos_z);
}
void ROSbot::move() {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0);  // Timeout of 2 seconds
  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x = +0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void ROSbot::move_forward(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    ROS_INFO_STREAM("Moving forward");
    ros::spinOnce();
    vel_msg.linear.x = 0.4;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void ROSbot::move_backwards(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    //ROS_INFO_STREAM("Moving backwards");
    ros::spinOnce();
    vel_msg.linear.x = -0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void ROSbot::turn(std::string clock, int n_secs) {
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(n_secs);

  double WZ = 0.0;
  if (clock == "clockwise") {
    //ROS_INFO_STREAM("Turning clockwise");
    WZ = -2.5;
  } else if (clock == "counterclockwise") {
    // ROS_INFO_STREAM("Turning counterclockwiseeee");
    WZ = 2.5;
  }

  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = WZ;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void ROSbot::stop() {
  //ROS_INFO_STREAM("Stopping the robot");
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

float ROSbot::get_pos(int param) {
  if (param == 1) {
    return this->pos_x;
  } else if (param == 2) {
    return this->pos_y;
  } else if (param == 3) {
    return this->pos_z;
  }
  return 0.0;
}

std::list<float> ROSbot::get_complete_pos() {
  std::list<float> coordinates({this->pos_x, this->pos_y, this->pos_z});
  return coordinates;
}

double ROSbot::get_time() {
  double time = ros::Time::now().toSec();
  return time;
}

float ROSbot::get_laser(int index) {
    if (index >= 0 && index < laser_range.size()) {
        return laser_range[index];
    } else {
        ROS_ERROR_STREAM("Index out of bounds: " << index);
        return -1.0; 
    }
}

std::unique_ptr<float[]> ROSbot::get_complete_laser() {
  std::unique_ptr<float[]> laser_range_cp(new float[laser_range.size()]);

  for (size_t i = 0; i < laser_range.size(); ++i) {
    laser_range_cp[i] = laser_range[i];
  }

  return laser_range_cp; 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_class_node");

  ROSbot rosbot;

  rosbot.move();

  float coordinate = rosbot.get_pos(1);

  ROS_INFO_STREAM(coordinate);

  return 0;
}