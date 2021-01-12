#ifndef LRF_CONTAINER_HPP
#define LRF_CONTAINER_HPP

#include <string>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//自作
#include "pcl_container.hpp"

class LRFContainer
{
  private:
    //cloud_xyzrgb_updatedにはpcl_inputが呼び出されるたびにlrfから得たpclの値が手に入る
    //XYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_updated;
    //XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_updated;
    
    //laserscanからpclに変換するのに必要(詳細不明)
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
    
    //subscriberのfixed frameの名前
    std::string sub_fixed_frame;

    //どの点群データの形式を扱うかのスイッチ
    //0 : xyzrgb デフォルト
    //1 : xyz
    int cloud_format;

    //rgbの場合、色を入れるかどうかのスイッチ
    bool color_sw;

    void callback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        //sensor_msgs::PointCloudからsensor_msgs::PointCloud2へ、sensor_msgs::PointCloud2からpclにして配信している。フィルタリングは配信後にしたほうが良さそう(二度手間すぎるため)
        sensor_msgs::PointCloud sm_cloud;
        sensor_msgs::PointCloud2 sm_cloud2;
        try
        {
            projector.transformLaserScanToPointCloud(sub_fixed_frame, *scan_in, sm_cloud, listener);
            sensor_msgs::convertPointCloudToPointCloud2(sm_cloud, sm_cloud2);
        }
        catch (tf::TransformException &e)
        {
            std::cout << e.what();
            return;
        }
        
        //当てはまる形式に変換
        if(cloud_format == 0){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(sm_cloud2, *cloud_xyzrgb);
            cloud_xyzrgb_updated = cloud_xyzrgb;
        }else if(cloud_format == 1){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(sm_cloud2, *cloud_xyz);
            cloud_xyz_updated = cloud_xyz;
        }
    }

  public:
    LRFContainer(ros::NodeHandle nh, std::string sub_fixed_frame_name, std::string sub_topic_name) : laser_sub(nh, sub_topic_name, 10), laser_notifier(laser_sub, listener, sub_fixed_frame_name, 10)
    {
        //laserscanからpclへの変換に必要(詳細不明)
        laser_notifier.registerCallback(boost::bind(&LRFContainer::callback, this, _1));
        laser_notifier.setTolerance(ros::Duration(0.01));
        sub_fixed_frame = sub_fixed_frame_name;
        //配信の設定
        //cloudの初期化
        cloud_xyzrgb_updated = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_xyz_updated = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_format = 0;
        color_sw = false;
    }

    //最新の情報にアップデート
    void update(){
        ros::spinOnce();
    }

    void format_change(int i){
        cloud_format = i;
    }

    void color_change(bool i){
        color_sw = i;
    }

    //最新のpc(point cloud)を提供する
    pcl::PointCloud<pcl::PointXYZRGB> pc_xyzrgb_latest()
    {
        return *cloud_xyzrgb_updated;
    }

    pcl::PointCloud<pcl::PointXYZ> pc_xyz_latest()
    {
        return *cloud_xyz_updated;
    }
};

#endif //LRF_CONTAINER_HPP