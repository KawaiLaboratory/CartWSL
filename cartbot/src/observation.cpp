#if 0
#else

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include "tf/transform_broadcaster.h"

#include "key_event.hpp"
#include "observation_lib.hpp"
#include "lrf_container.hpp"
#include "pcl_container.hpp"
#include "extension_lib.hpp"


int main(int argc, char **argv)
{
    //Hello ROS !!
    ROS_INFO_STREAM("observation start");
    ros::init(argc, argv, "cartbot_observation");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    bool running = true;

    //LRFContainer
    std::string lrf_sub_fixed_frame = "/laser";
    std::string lrf_sub_topic = "/scan";
    LRFContainer lrfc(nh, lrf_sub_fixed_frame, lrf_sub_topic);

    //point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lrf(new pcl::PointCloud<pcl::PointXYZRGB>);

    //LRF配信
    std::string lrf_pub_topic = "lrf_observation";
    uint32_t queue_size = 1;
    ros::Publisher pub_lrf = nh.advertise<sensor_msgs::PointCloud2>(lrf_pub_topic, queue_size);

    //LRFの特徴をpythonへ送信
    std::string lrf_send_to_python_pub = "lrf_feature";
    ros::Publisher pub_lrf_send_to_python = nh.advertise<std_msgs::String>(lrf_send_to_python_pub, 1);

    //PCD保存番号
    int cs_pcd_number = 0;
    int s_pcd_number = 0;

    //保存フォルダ確認
    std::string env_path=std::getenv("CARTBOT_FEATURES_DATA_FOLDER");
    std::string features_file = featureFilePath(0,env_path);
    std::string load_path = "";
    std::string extract_path = "";
    std::string cs_path = "";
    ROS_INFO_STREAM("data folder:"+env_path);
    int load_number = 0;

    //検知範囲指定
    float min_x = 0, max_x = 3.0f, min_y = -0.5f, max_y = 0.5f;

    //コマンド
    int command = -1;
    int cs = 0;//c,dのスイッチ
    int cv = 0;//vのスイッチ
    int parts_sw = 2;
    int clustring_sw = 0;

    //タイマー
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

    while (ros::ok() && running)
    {
        /* 
        //計測終了時刻を保存
        end = std::chrono::system_clock::now();
        // 要した時間を計算
        dur = end - start;
        //msecで値を用意 
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
        //msec表示
        std::cout << "sleep:" << msec << "ms" << std::endl;
        */
        //----------------process計測----------------------------
        //start = std::chrono::system_clock::now();

        //計測値収容用pointcloud配列
        std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > cloud_cluster;

        //LRF所得値を得る&いらない範囲を除去
        lrfc.update();
        *cloud_lrf = lrfc.pc_xyzrgb_latest();
        *cloud_lrf = passThroughContainer(cloud_lrf, "y", min_y, max_y);
        *cloud_lrf = passThroughContainer(cloud_lrf, "x", min_x, max_x);
        cloud_cluster = clusteringContainer(cloud_lrf);

        //------------------------visualモードかどうか--------------------
        if(cv == 0){
            //観測値にクラスタリングを使うか切り替えられる
            if(clustring_sw == 0){
                int max_point_number = 0;
                //クラスタリングで分けたものを利用
                for (int i = 0; i < cloud_cluster.size(); i++)
                {//クラスタリング後、最も点の多い塊を保存対象物とする。このモードは赤色点群
                    if(cloud_cluster[i]->size() > max_point_number){
                        max_point_number = cloud_cluster[i]->size();
                        cloud_lrf = cloud_cluster[i];
                        for(int l = 0; l < cloud_lrf->points.size(); l++)
                        {
                            cloud_lrf->points[l].r = 255;
                            cloud_lrf->points[l].g = 0;
                            cloud_lrf->points[l].b = 0;
                        }
                    }
                }
            }else if(clustring_sw == 1){
                for (int i = 0; i < cloud_lrf->points.size(); i++)
                {//クラスタリングされたデータすべてを保存する。このモードは黄色点群。初期設定。
                    cloud_lrf->points[i].r = 255;
                    cloud_lrf->points[i].g = 255;
                    cloud_lrf->points[i].b = 0;
                }
            }else if(clustring_sw == 2){
                //LRFの取得地をそのまま利用。このモードは緑色点群
                for (int i = 0; i < cloud_lrf->points.size(); i++)
                {
                    cloud_lrf->points[i].r = 0;
                    cloud_lrf->points[i].g = 255;
                    cloud_lrf->points[i].b = 0;
                }
            }
        }else{
            //pcdファイルを観覧
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (env_path + load_path + "/o_lrf_"+std::to_string(load_number)+".pcd", *cloud_lrf) == -1){ //* load the file
                //PCL_ERROR ("Couldn't read file .pcd \n");
            }
        }
        
        //-------------------配信----------------------------------------
        pc_pub(cloud_lrf, pub_lrf, lrf_sub_fixed_frame);

        //-------------------コマンド----------------------------------------
        command = key_event_char();
        if (command != -1)
        {
            if (command == 113) //q(quit) 終了
            {
                running = false;
            }
            else if (command == 114) //r(remove) features.csvを削除する
            {
                remove((features_file).c_str());
                ROS_INFO_STREAM(".csv delete");
            }
            else if (command == 101) //e(extract)　features.csvに追記を行う
            {
                ROS_INFO_STREAM("extraction");
                std::cout << "右のパス以下のファイル保存フォルダまでのパスを入力：" << env_path << std::endl;
                std::cout << "最初に/が必要、最後に/は必要なし" << std::endl;
                std::cin >> extract_path;
                //ファイルを開く
                std::ofstream outputfile(features_file,std::ios::app);
                for(int i = 0; checkExistence(env_path+extract_path+"/o_lrf_"+std::to_string(i)+".pcd");i++){
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::io::loadPCDFile (env_path+extract_path+"/o_lrf_"+std::to_string(i)+".pcd", *cloud);
                    features feature = featureExtraction(cloud);
                    
                    outputfile << std::to_string(feature.girth) + "," + std::to_string(feature.width) + "," + std::to_string(feature.depth) << std::endl;
                }
                outputfile.close();
                std::cout << "e end" << std::endl;
            }
            else if(command == 99) //c(continuity save) 連続保存
            {
                if(cs == 0){
                    ROS_INFO_STREAM("continuity save");
                    cs = 1;
                }
            }
            else if(command == 100)//d(continuity save end)  モードの終了
            {
                ROS_INFO_STREAM("c/v end");
                cs = 0;
                cv = 0;
            }
            else if(command == 109)//m(modify) 取得範囲の修正
            {
                std::cout << "Please input detection range : min_x max_x min_y max_y" << std::endl;
                std::cin >> min_x >> max_x >> min_y >> max_y;
            }
            else if(command == 118)//v(visualize) pcdファイルの観覧モード
            {
                ROS_INFO_STREAM("visualize");
                std::cout << "右のパス以下のファイル保存フォルダまでのパスを入力：" << env_path << std::endl;
                std::cout << "最初に/が必要、最後に/は必要なし" << std::endl;
                std::cin >> load_path;
                cv = 1;
            }
            else if(command == 44){//, visualモードにおいて、pcdファイルの選択を前の物にする
                if(load_number > 0){
                    load_number--;
                }
            }
            else if(command == 46){//. visualモードにおいて、pcdファイルの選択を後の物にする
                load_number++;
            }
            else if(command == 111){//o clustring observation クラスタリングを使って観測するかどうか
                ROS_INFO_STREAM("clusring observation");
                if(cs == 0){
                    clustring_sw++;
                    if (clustring_sw == 3){
                        clustring_sw = 0;
                    }
                    ROS_INFO_STREAM(clustring_sw);
                }
            }
            else if(command == 112){//p
                std::cout << "測定する部位を指定します。コマンドeを押した際の保存先が変わります。" << env_path << std::endl;
                std::cout << "0:胴 1:足 2:腰" << std::endl;
                std::cout << "現在:" << parts_sw << std::endl;
                std::cin >> parts_sw;
                features_file = featureFilePath(parts_sw,env_path);
            }
        }

        //c (連続保存)のスイッチ
        if(clustring_sw == 2){
            if(cs != 0){
                if (cs == 1){
                    cs_pcd_number=0;
                    if((cs_path = c_s(env_path+"/observation")) != ""){
                        cs = 2;
                    }else{
                        cs = 0;
                    }
                }else{
                    if(cloud_lrf->points.size()>0){
                        save_cloud_observation(cs_path,cloud_lrf,cs_pcd_number);
                        cs_pcd_number++;
                    }
                }
            }
        }else if(clustring_sw == 1){
            if(cs != 0){
                if (cs == 1){
                    cs_pcd_number=0;
                    if((cs_path = c_s(env_path+"/observation")) != ""){
                        cs = 2;
                    }else{
                        cs = 0;
                    }
                }else{
                    if(cloud_lrf->points.size()>0){
                        for (int i = 0; i < cloud_cluster.size(); i++)
                        {//クラスタリング後、最も点の多い塊を保存対象物とする。
                            cloud_lrf = cloud_cluster[i];
                            save_cloud_observation(cs_path,cloud_lrf,cs_pcd_number);
                            cs_pcd_number++;
                        }
                    }
                }
            }
        }
        
        /*
        end = std::chrono::system_clock::now();       // 計測終了時刻を保存
        dur = end - start;        // 要した時間を計算
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
        //std::cout << "process:" << msec << "ms" << std::endl;
        //sleep計測
        start = std::chrono::system_clock::now();
        */

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

#endif