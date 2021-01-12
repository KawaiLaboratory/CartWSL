#include <ros/ros.h>
#include <string>

//self made
#include "lrf_container.hpp"
#include "pcl_container.hpp"


ros::Publisher pub_lrf;
ros::Publisher pub_lrf_clustring;

int main(int argc, char **argv)
{
  //Hello ROS !!
  ROS_INFO_STREAM("cartbot start !!");
  ros::init(argc, argv, "cartbot");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);
  printf("AED");

  //lrf container
  std::string lrf_sub_topic = "/scan";
  std::string lrf_sub_fixed_frame = "/laser";
  LRFContainer lrfc(nh, lrf_sub_fixed_frame, lrf_sub_topic);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lrf(new pcl::PointCloud<pcl::PointXYZ>);
  lrfc.format_change(1);
  //LRF配信用
  uint32_t queue_size = 10;
  std::string lrf_pub_topic = "lrf_output";
  pub_lrf = nh.advertise<sensor_msgs::PointCloud2>(lrf_pub_topic, queue_size);
  std::string lrf_pub_clustring_topic = "lrf_clustring";
  pub_lrf_clustring = nh.advertise<sensor_msgs::PointCloud2>(lrf_pub_clustring_topic, queue_size);
  //LRFの特徴をpythonへ送信
  std::string lrf_send_to_python_pub = "lrf_feature";
  ros::Publisher pub_lrf_send_to_python = nh.advertise<std_msgs::String>(lrf_send_to_python_pub, queue_size);
//  ros::Publisher pub_lrf_send_to_python = nh.advertise<std_msgs::Float32MultiArray>(lrf_send_to_python_pub, queue_size);

  while (ros::ok())
  {
    lrfc.update();
    *cloud_lrf = lrfc.pc_xyz_latest();
    pc_pub(cloud_lrf, pub_lrf,lrf_sub_fixed_frame);
    *cloud_lrf = passThroughContainer(cloud_lrf,"x",-1.0,4.0);
    *cloud_lrf = passThroughContainer(cloud_lrf,"y",-4.0,4.0);
    *cloud_lrf = humanDetectionContainer(cloud_lrf,pub_lrf_send_to_python);
    pc_pub(cloud_lrf, pub_lrf_clustring,lrf_sub_fixed_frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}