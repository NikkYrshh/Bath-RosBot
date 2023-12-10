#include "rosbot_bath/rosbot_class.h"
#include <ros/ros.h>

#include <string>

using namespace std;

// New implemented class
class RosbotMove {
    
public:
    
  RosbotMove(string a) { left_or_right = a; };
  string left_or_right;
    
  // Inherit RosbotClass from the file rosbot_class.h
  ROSbot rosbot;
  void avoid();
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "Rosbot_move_node");
}
