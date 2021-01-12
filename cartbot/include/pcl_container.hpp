#ifndef PCL_CONTAINER_HPP
#define PCL_CONTAINER_HPP

#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
//PCL Library
#define PCL_NO_PRECOMPILE
//directly under
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
//filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//features
#include <pcl/features/normal_3d.h>
//kdtree
#include <pcl/kdtree/kdtree.h>
//sample_consensus
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//conversions
#include <pcl_conversions/pcl_conversions.h>

//------------------------------
//-----pc_pub-------------------
//------------------------------
void pc_pub(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Publisher pub, std::string header_frame_id)
{
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = header_frame_id;
  pub.publish(output);
}
void pc_pub(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher pub, std::string header_frame_id)
{
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = header_frame_id;
  pub.publish(output);
}

void save_cloud_observation(std::string folder_path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int pcd_number)
{
    std::cout << "PointCloud representing the Cluster: " << cloud->points.size() << " data points." << std::endl;
    std::stringstream ss;
    ss << folder_path << "o_lrf_" << pcd_number << ".pcd"; //クラスター毎に名前を変化
    pcl::io::savePCDFileBinary(ss.str(), *cloud);
}

//------------------------------
//-----passThroughContainer-----
//------------------------------
pcl::PointCloud<pcl::PointXYZ> passThroughContainer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string direction, float min, float max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    //direction = x or y, z
    pass.setFilterFieldName(direction);
    pass.setFilterLimits(min, max);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    return *cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> passThroughContainer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string direction, float min, float max)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    //direction = x or y, z
    pass.setFilterFieldName(direction);
    pass.setFilterLimits(min, max);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    return *cloud_filtered;
}

//LRFの特徴をまとめる
struct features
{
    float girth;
    float width;
    float depth;
    float center_point_x;
    float center_point_y;
    float center_point_z;
};

void msgPublisher(ros::Publisher pub, std::string s)
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << s;
    msg.data = ss.str();
    pub.publish(msg);
}


//------------------------------
//---二点間距離計算-------------
//------------------------------
float distanceBetweenTwoPointsIn2D(pcl::PointXYZRGB a, pcl::PointXYZRGB b)
{
    return std::sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
}
float distanceBetweenTwoPointsIn2D(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return std::sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
}


//------------------------------
//---奥行き測定-----------------
//------------------------------
float depthMeasurement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointXYZRGB p1 = cloud->points[0], pm = cloud->points[cloud->points.size() - 1], L1, L2;
    float L2TL2 = 0, L1TL1 = 0, L1TL2 = 0, depth_max = 0, depth = 0;
    L1.x = pm.x - p1.x;
    L1.y = pm.y - p1.y;
    L1TL1 = L1.x * L1.x + L1.y * L1.y;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        L2.x = cloud->points[i].x - p1.x;
        L2.y = cloud->points[i].y - p1.y;
        L2TL2 = L2.x * L2.x + L2.y * L2.y;
        L1TL2 = L1.x * L2.x + L1.y * L2.y;
        depth = (L2TL2 * L1TL1 - std::pow(L1TL2, 2)) / L1TL1;
        if (depth > depth_max)
        {
            depth_max = depth;
        }
    }
    return depth_max;
}
float depthMeasurement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ p1 = cloud->points[0], pm = cloud->points[cloud->points.size() - 1], L1, L2;
    float L2TL2 = 0, L1TL1 = 0, L1TL2 = 0, depth_max = 0, depth = 0;
    L1.x = pm.x - p1.x;
    L1.y = pm.y - p1.y;
    L1TL1 = L1.x * L1.x + L1.y * L1.y;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        L2.x = cloud->points[i].x - p1.x;
        L2.y = cloud->points[i].y - p1.y;
        L2TL2 = L2.x * L2.x + L2.y * L2.y;
        L1TL2 = L1.x * L2.x + L1.y * L2.y;
        depth = (L2TL2 * L1TL1 - std::pow(L1TL2, 2)) / L1TL1;
        if (depth > depth_max)
        {
            depth_max = depth;
        }
    }
    return depth_max;
}

//humanDetectionContainerはmain.cppで使われる。
pcl::PointCloud<pcl::PointXYZ> humanDetectionContainer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher pub)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.06f); //mの余裕持ち
    ec.setMinClusterSize(10);      //最小のクラスターの値を設定
    ec.setMaxClusterSize(500);     //最大のクラスターの値を設定
    ec.setSearchMethod(tree);      //検索に使用する手法を指定
    ec.setInputCloud(cloud);       //点群を入力
    ec.extract(cluster_indices);   //クラスター情報を出力

    int feature_number = 0;
    std::vector<features> features_vector;

    int color_number = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) //クラスターを1塊ごと出力
    {
        pcl::PointXYZ start_point, current_point, before_point;
        features feature;
        feature.girth = 0;
        feature.width = 0;
        feature.depth = 0;
        int j = 0;
        float center_point_x = 0, center_point_y = 0, center_point_z = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);

            if (j == 0)
            {
                start_point = cloud->points[*pit];
                current_point = start_point;
            }
            else
            {
                //胴回り(girth)を計測中
                before_point = current_point;
                current_point = cloud->points[*pit];
                feature.girth += distanceBetweenTwoPointsIn2D(current_point, before_point);
            }

            center_point_x += cloud->points[*pit].x;
            center_point_y += cloud->points[*pit].y;
            center_point_z += cloud->points[*pit].z;

            //確認用
            //std::cout << cloud->points[*pit] << std::endl;

            j++;
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //幅(width)を始点(start_point)から終点(current_point)までの距離で図る
        feature.width = distanceBetweenTwoPointsIn2D(current_point, start_point);

        //奥行き(depth)を図る
        feature.depth = depthMeasurement(cloud_cluster);

        feature.center_point_x = center_point_x / (float)j;
        feature.center_point_y = center_point_y / (float)j;
        feature.center_point_z = center_point_z / (float)j;

        //確認用識別カラーナンバー
        color_number++;
        //std::cout<< feature.girth<<" "<<feature.width<<" "<<feature.depth<<std::endl;
        features_vector.push_back(feature);
    }

    std::string send_msg = "";
    for (int i = 0; i != features_vector.size(); ++i)
    {
        send_msg += (std::to_string(features_vector[i].girth) + "," + std::to_string(features_vector[i].width) + "," + std::to_string(features_vector[i].depth) + "," + std::to_string(features_vector[i].center_point_x) + "," + std::to_string(features_vector[i].center_point_y) + "," + std::to_string(features_vector[i].center_point_z) + ";");
    }
    msgPublisher(pub, send_msg);
    return *cloud;
}



//clusteringContainerはobservation.cppのみで使われる。
std::vector <pcl::PointCloud <pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr> > clusteringContainer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.06f); //mの余裕持ち
    ec.setMinClusterSize(10);      //最小のクラスターの値を設定
    ec.setMaxClusterSize(200);     //最大のクラスターの値を設定
    ec.setSearchMethod(tree);      //検索に使用する手法を指定
    ec.setInputCloud(cloud);       //点群を入力
    ec.extract(cluster_indices);   //クラスター情報を出力

    std::vector <pcl::PointCloud <pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > res;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) //クラスターを1塊ごと出力
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        res.push_back(cloud_cluster);
    }

    return res;
}

#endif //PCL_CONTAINER_HPP