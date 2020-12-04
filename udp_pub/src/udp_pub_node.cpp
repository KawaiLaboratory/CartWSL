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
  std::ofstream fs("~/catkin_ws/src/udp_pub/csvs/"+std::to_string(now.toSec())+".csv");  // CSVファイル生成
  fs << "l, theta" << std::endl;

  for(int i = 0; i < 726; i++) {
    l = scan->ranges[i];
    theta = scan->angle_min + scan->angle_increment * i;

    if(theta > scan->angle_max){
      break;
    }else{
      fs << l << "," << theta << std::endl;
    }
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
  ros::Rate sleep_rate(20);

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
  pub = n.advertise<std_msgs::Float32MultiArray>("/u_cbf", 1000);

  std_msgs::Float32MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = 0;
  msg.data[1] = 0.1;

  pub.publish(msg);
  while(cnt < 100){
    sleep_rate.sleep();
    ros::spinOnce();
  }
 return 0;
}
