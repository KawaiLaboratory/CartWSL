//for LRF
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
//for CSV
#include "fstream"

int cnt = 0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  double l = 0.0;     //得られた最近点までの距離[m]
  double theta = 0.0; //得られた最近点の角度[rad]
  ros::Time now = ros::Time::now();
  std::ofstream fs(std::to_string(now.toSec())+".csv");  // CSVファイル生成
  fs << "l, theta,range_max" << std::endl;

  for(int i = 0; i < 726; i++) {
    l = scan->ranges[i];
    theta = scan->angle_min + scan->angle_increment * i;

    fs << l << "," << theta << "," << scan->range_max << std::endl;
  }
  fs.close();
  cnt += 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udp_pub");
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Rate rate(20);
  ros::Rate sleep(0.1);

  sleep.sleep();

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
  pub = n.advertise<std_msgs::Float32MultiArray>("/u_cbf", 1000);

  std_msgs::Float32MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = 0;
  msg.data[1] = -0.1;

  ros::Time start = ros::Time::now();
  ros::Time now = ros::Time::now();

  while((now-start).toSec() < 80){
    pub.publish(msg);

    rate.sleep();
    ros::spinOnce();

    now = ros::Time::now();
  }

  msg.data[1] = 0;
  pub.publish(msg);
  ROS_INFO("csv size is %d", cnt);
  return 0;
}
