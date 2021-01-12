#ifndef OBSERVATION_LIB_HPP
#define OBSERVATION_LIB_HPP

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include "tf/transform_broadcaster.h"

#include "lrf_container.hpp"
#include "pcl_container.hpp"
#include "key_event.hpp"
#include "extension_lib.hpp"

std::string c_s(std::string folder_path)
{
    time_t now = std::time(nullptr);
    struct tm* localNow = std::localtime(&now);
    std::string res = "";

    if(checkExistence(folder_path)){
        std::string date_folder_name = "/"+std::to_string(1900+localNow->tm_year)+"_"+std::to_string(localNow->tm_mon+1)+"_"+std::to_string(localNow->tm_mday);
        if(checkExistence(folder_path+date_folder_name)){
            for(int i = 0; 1; i++)
            {
                if(!checkExistence(folder_path+date_folder_name+"/"+std::to_string(i))){
                    res = folder_path+date_folder_name+"/"+std::to_string(i)+"/";
                    createFolder(res);
                    break;
                }
            }
        }
        else
        {
            res = folder_path+date_folder_name+"/0/";
            createFolder(folder_path+date_folder_name);
            createFolder(res);
        }
    }
    else
    {
        std::cout<<"path is incorrect"<<std::endl;
    }

    return res;
}

std::string featureFilePath(int i,std::string env_path){
    std::string path = "";
    if(i == 0){
        path = env_path + "/torso_features_data.csv";
    }else if(i == 1){
        path = env_path + "/leg_features_data.csv";
    }else if(i == 2){
        path = env_path + "/hip_features_data.csv";
    }
    return path;
}

features featureExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    pcl::PointXYZRGB start_point, current_point, before_point;
    features feature;
    feature.girth = 0;
    feature.width = 0;
    feature.depth = 0;
    //特徴データ保存
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (i == 0)
        {
            start_point = cloud->points[i];
            current_point = start_point;
        }
        else
        {
            before_point = current_point;
            current_point = cloud->points[i];
            feature.girth += distanceBetweenTwoPointsIn2D(current_point, before_point);
        }
    }
    feature.width = distanceBetweenTwoPointsIn2D(current_point, start_point);

    //奥行き(depth)を図る
    feature.depth = depthMeasurement(cloud);

    return feature;
}

void observationCommand(){}


#endif //OBSERVATION_LIB_HPP