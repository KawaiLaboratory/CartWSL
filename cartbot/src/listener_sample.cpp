#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

ros::Publisher pub_lrf;

void test_callback(const std_msgs::Float32MultiArray::ConstPtr& array){
  //status : 0 or 1
  std::cout << "judgement: " <<array->data[0] << std::endl;
  //x 前後
  std::cout << "x: " << array->data[1] << std::endl;
  //y 左右
  std::cout << "y: " << array->data[2] << std::endl;
}


int main(int argc, char **argv)
{
  //Hello ROS !!
  ROS_INFO_STREAM("listener_sample start");
  ros::init(argc, argv, "listener_sample");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);
  ros::Subscriber sub = nh.subscribe("status", 1, test_callback);
  ros::spin();
  return 0;
}