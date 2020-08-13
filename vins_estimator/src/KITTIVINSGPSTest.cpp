//
// Created by sqg on 2020/6/20.
//

/// kitti--stereo+imu+gps

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;    /// 估计器全局变量


/**
 * @brief   load datapath and get image/gps/imu(gps与imu时间戳相同) time stamp
 * @param   dataPath
 * @return  vector<int> TimeList
 */
vector<double> stamp_load(string dataPath, string Type)
{
    FILE* file;

    if(file == NULL)
    {
        printf("cannot find timestamps.txt \n");
        ROS_BREAK();
    }

    if (Type=="image")
        file = std::fopen((dataPath + "image_00/timestamps.txt").c_str(), "r");
    else if (Type=="gps"||Type=="imu")
        file = std::fopen((dataPath + "oxts/timestamps.txt").c_str() , "r");
    else
    {
        printf("please input correct parameter! \n");
        ROS_BREAK();
    }

    vector<double> TimeList;    /// 存放image/gps的时间戳
    int year, month, day;
    int hour, minute;
    double second;

    while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
    {
        // printf("%lf\n", second);
        TimeList.push_back(hour * 60 * 60 + minute * 60 + second);
    }
    std::fclose(file);
    return TimeList;
}

/// 接下来主要看怎么写回调函数或其它实现方式来完成 stereo+imu 的融合

/// 目前　vins-gps/vins_estimator/src/factor/imu_factor.h　114行出错（2020/6/22）

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ///ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
    ///ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);
    ///ros::Publisher pubIMU = n.advertise<sensor_msgs::Imu>("/imu", 1000);
    ros::Publisher pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);

    if(argc != 3)
    {
        printf("please intput: rosrun vins kitti_gps_test [config file] [data folder] \n"
               "for example: rosrun vins kitti_gps_test "
               "~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml "
               "/media/tony-ws1/disk_D/kitti/2011_10_03/2011_10_03_drive_0027_sync/ \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    string sequence = argv[2];
    printf("read sequence: %s\n", argv[2]);
    string dataPath = sequence + "/";

    /// 加载配置文件
    readParameters(config_file);
    estimator.setParameter();
    registerPub(n);

    /// 加载图片时间戳
    FILE* file;
    file = std::fopen((dataPath + "image_00/timestamps.txt").c_str() , "r");    // 打开00图片的时间戳文件
    if(file == NULL){
        printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
        ROS_BREAK();
    }

    vector<double> imageTimeList;    // 存放格式化后的时间戳文件
    int year, month, day;
    int hour, minute;
    double second;

    while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
    {
        //printf("%lf\n", second);
        imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
    }
    std::fclose(file);

    /// 加载IMU/GPS时间戳
    vector<double> IMUTimeList;    // 存放gps时间戳序列
    {
        FILE* file;
        file = std::fopen((dataPath + "oxts/timestamps.txt").c_str() , "r");
        if(file == NULL){
            printf("cannot find file: %soxts/timestamps.txt \n", dataPath.c_str());
            ROS_BREAK();
            return 0;
        }

        /// 将IMU时间戳存入IMUTimeList
        int year, month, day;
        int hour, minute;
        double second;
        while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
        {
            //printf("%lf\n", second);
            IMUTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
        }
        std::fclose(file);
    }

    /// 配置输出路径
    FILE* outFile;
    outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
    if(outFile == NULL)
        printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());


    ///*****************************主循环：将图片/IMU加入估计器***********************************///
    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;
    double baseTime;

    for (size_t i = 0; i < imageTimeList.size(); i++)
    {
        if(ros::ok())
        {
            /// 初始化开始时间
            if(imageTimeList[0] < IMUTimeList[0])
            {
                baseTime = imageTimeList[0];
                printf("base time is %f\n", baseTime);
            }
            else
            {
                baseTime = IMUTimeList[0];
                printf("base time is %f\n", baseTime);
            }

            ///*******************************1.将图片加入到估计器******************************///
            printf("process image %d\n", (int)i);
            stringstream ss;
            ss << setfill('0') << setw(10) << i;
            leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";    /// 格式化图片路径
            rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";   /// 格式化图片路径
            //printf("%s\n", leftImagePath.c_str() );
            //printf("%s\n", rightImagePath.c_str() );

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            //sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
            //imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
            //pubLeftImage.publish(imLeftMsg);

            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            //sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
            //imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
            //pubRightImage.publish(imRightMsg);

            double imgTime = imageTimeList[i] - baseTime;    /// 图片的绝对时间

            estimator.inputImage(imgTime, imLeft, imRight);

            ///*******************************2.将IMU加入到估计器******************************///
            printf("process IMU %d\n", (int)i);

            FILE* IMUFile;
            string IMUFilePath = dataPath + "oxts/data/" + ss.str() + ".txt";    /// 格式化IMU数据路径
            IMUFile = std::fopen(IMUFilePath.c_str() , "r");
            if(IMUFile == NULL){
                printf("cannot find file: %s\n", IMUFilePath.c_str());
                ROS_BREAK();
                return 0;
            }

            double lat, lon, alt, roll, pitch, yaw;
            double vn, ve, vf, vl, vu;
            double ax, ay, az, af, al, au;
            double wx, wy, wz, wf, wl, wu;
            double pos_accuracy, vel_accuracy;
            double navstat, numsats;
            double velmode, orimode;

            fscanf(IMUFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
            // printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);  /// 注释
            fscanf(IMUFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
            // printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);  /// 注释
            fscanf(IMUFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
            // printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);  /// 注释
            fscanf(IMUFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
            // printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);  /// 注释
            fscanf(IMUFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
            // printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
            // 	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);  /// 注释

            std::fclose(IMUFile);

            Vector3d linearAcceleration(ax, ay, az);
            Vector3d angularVelocity(wx, wy, wz);

            double IMUTime = IMUTimeList[i] - baseTime;    /// IMU的绝对时间

            estimator.inputIMU(IMUTime, linearAcceleration, angularVelocity);

            ///**************************3.读取GPS数据并将其发布*************************///
			sensor_msgs::NavSatFix gps_position;
			gps_position.header.frame_id = "NED";
			gps_position.header.stamp = ros::Time(imgTime);
			gps_position.status.status = navstat;
			gps_position.status.service = numsats;
			gps_position.latitude  = lat;
			gps_position.longitude = lon;
			gps_position.altitude  = alt;
			gps_position.position_covariance[0] = pos_accuracy;

			// printf("pos_accuracy %f \n", pos_accuracy);
			pubGPS.publish(gps_position);

            ///*******************************4.输出结果******************************///
            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            if(outFile != NULL)
                fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",
                         pose(0,0), pose(0,1), pose(0,2),pose(0,3),
                         pose(1,0), pose(1,1), pose(1,2),pose(1,3),
                         pose(2,0), pose(2,1), pose(2,2),pose(2,3));

            //cv::imshow("leftImage", imLeft);
            //cv::imshow("rightImage", imRight);
            //cv::waitKey(2);
        }
        else
            break;
    }
    if(outFile != NULL)
        fclose (outFile);
    return 0;
}



