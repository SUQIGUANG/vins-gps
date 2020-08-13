/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

/// kitti--stereo+gps

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;
ros::Publisher pubGPS;


/**
 * @brief KITTIGPSTest主函数加载 kitti--stereo+gps 相关文件和计算
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	/// 发布GPS话题，可由其他节点订阅
	pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);

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

	/// load image list    处理图片时间戳
	FILE* file;
	file = std::fopen((dataPath + "image_00/timestamps.txt").c_str() , "r");    // 打开00图片的时间戳文件
	if(file == NULL){
	    printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
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


	/// load gps list   　处理gps时间戳　　 oxts是一个IMU+GNSS传感器组合
	vector<double> GPSTimeList;    // 存放gps时间戳序列
	{
		FILE* file;
		file = std::fopen((dataPath + "oxts/timestamps.txt").c_str() , "r");
		if(file == NULL){
		    printf("cannot find file: %soxts/timestamps.txt \n", dataPath.c_str());
		    ROS_BREAK();
		    return 0;          
		}

		/// 将gps时间戳存入GPSTimeList
		int year, month, day;
		int hour, minute;
		double second;
		while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
		{
			//printf("%lf\n", second);
		    GPSTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
		}
		std::fclose(file);
	}

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

	FILE* outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vo.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	double baseTime;

	for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
		    /// 初始化开始时间
			if(imageTimeList[0] < GPSTimeList[0])
            {
                baseTime = imageTimeList[0];
                printf("base time is %f\n", baseTime);
            }
			else
            {
                baseTime = GPSTimeList[0];
                printf("base time is %f\n", baseTime);
            }

			printf("process image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(10) << i;
			leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";    /// 格式化图片路径
			rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";   /// 格式化图片路径
			printf("%s\n", leftImagePath.c_str() );
			printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );

			double imgTime = imageTimeList[i] - baseTime;    /// 图片的绝对时间

			// load gps
			FILE* GPSFile;
			string GPSFilePath = dataPath + "oxts/data/" + ss.str() + ".txt";    /// 格式化gps数据路径
			GPSFile = std::fopen(GPSFilePath.c_str() , "r");
			if(GPSFile == NULL){
			    printf("cannot find file: %s\n", GPSFilePath.c_str());
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

			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
			// printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);  /// 注释
			fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
			// printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);  /// 注释
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
			// printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);  /// 注释
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
			// printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);  /// 注释
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
			// printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
			// 	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);  /// 注释

			std::fclose(GPSFile);

			sensor_msgs::NavSatFix gps_position;
			gps_position.header.frame_id = "NED";
			gps_position.header.stamp = ros::Time(imgTime);
			gps_position.status.status = navstat;
			gps_position.status.service = numsats;
			gps_position.latitude  = lat;
			gps_position.longitude = lon;
			gps_position.altitude  = alt;
			gps_position.position_covariance[0] = pos_accuracy;

			// printf("pos_accuracy %f \n", pos_accuracy);  /// 注释
			pubGPS.publish(gps_position);



			estimator.inputImage(imgTime, imLeft, imRight);

			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",
				         pose(0,0), pose(0,1), pose(0,2),pose(0,3),
				         pose(1,0), pose(1,1), pose(1,2),pose(1,3),
				         pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			// cv::imshow("leftImage", imLeft);
			// cv::imshow("rightImage", imRight);
			// cv::waitKey(2);
		}
		else
			break;
	}
	if(outFile != NULL)
		fclose (outFile);
	return 0;
}

