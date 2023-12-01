#ifndef ROSBOT_CLASS_H
#define ROSBOT_CLASS_H

#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

class ROSbot {
 private:
  ros::NodeHandle nh;

  std::vector<float> laser_range;
  // subscribers
  ros::Subscriber laser_sub;
  ros::Subscriber odom_sub;
  // velocity pub and msg
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel_msg;

  // topics
  std::string laser_topic;
  std::string vel_topic;
  std::string odom_topic;

  // pose x,y,z
  float pos_x;
  float pos_y;
  float pos_z;

  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

 public:
  ROSbot();
  void move();
  void move_forward(int time);
  void move_backwards(int time);
  void turn(std::string clock, int time);
  void stop();
  float get_pos(int param);
  std::list<float> get_complete_pos();
  double get_time();
  float get_laser(int index);
  std::unique_ptr<float[]> get_complete_laser();
};

#endif  // ROSBOT_CLASS_H