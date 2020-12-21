// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               Livox@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <ceres/ceres.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <std_srvs/Trigger.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
/*
#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "loam_horizon/tic_toc.h"
/home/tr/catkin_ws/src/a-loam-not/include/aloam_velodyne/common.h
/home/tr/catkin_ws/src/a-loam-not/include/aloam_velodyne/tic_toc.h
*/
#include <pcl/outofcore/outofcore.h>

#include <pcl/outofcore/outofcore_impl.h>
//new******************************************************
//ros::Publisher pubColorMap;
std::list<sensor_msgs::PointCloud2ConstPtr> CloudRegisteredBuf;
std::list<sensor_msgs::PointCloud2ConstPtr>::iterator Clouditer;
std::list<sensor_msgs::Image::ConstPtr> ImageBuf;
std::list<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::list<nav_msgs::Odometry::ConstPtr>::iterator Odoiter;
std::list<nav_msgs::Odometry::ConstPtr>::iterator Odoiter1;
std::mutex mBuf;
cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
Eigen::Quaterniond q_wodom_last(1, 0, 0, 0);
Eigen::Vector3d t_wodom_last(0, 0, 0);
Eigen::Quaterniond q_wodom_next(1, 0, 0, 0);
Eigen::Vector3d t_wodom_next(0, 0, 0);
Eigen::Quaterniond q_wodom_cur(1, 0, 0, 0);
Eigen::Vector3d t_wodom_cur(0, 0, 0);
int image_use_count = 0;
pcl::PointCloud<pcl::PointXYZRGB>  totalCloud;
pcl::PointCloud<pcl::PointXYZ> sumCloud;

std::vector<Eigen::Vector3d> pc_temp;
ros::Publisher pubcloud_test;
ros::Publisher pubcloud_all;
ros::Publisher pubimg_test;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
//*********************************************************

/*
void Undistort_img(Mat map1, Mat map2, char *path)
{
	Mat img1, img2;
	img1 = imread(path);
	if (img1.empty()) cout << "Cannot open the image" << endl;
	
}*/
void loadCalibrationData(cv::Mat &R_rect_00,cv::Mat &RT) {
    RT.at<double>(0, 0) = 7.967514e-03;
    RT.at<double>(0, 1) = -9.999679e-01;
    RT.at<double>(0, 2) = -8.462264e-04;
    RT.at<double>(0, 3) = -1.377769e-02;
    RT.at<double>(1, 0) = -2.771053e-03;
    RT.at<double>(1, 1) = 8.241710e-04;
    RT.at<double>(1, 2) = -9.999958e-01;
    RT.at<double>(1, 3) = -5.542117e-02;
    RT.at<double>(2, 0) = 9.999644e-01;
    RT.at<double>(2, 1) = 7.969825e-03;
    RT.at<double>(2, 2) = -2.764397e-03;
    RT.at<double>(2, 3) = -2.918589e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;
//extrinsic
//0.007967514  -0.999679  -0.00485745  0.0400794
//0.0642422  0.0048885  -0.997922  0.0278129
//0.997934  0.000326217  0.0642445  -0.0291681
//0  0  0  1
//tr
// data: [7.967514e-03,  -9.999679e-01, -8.462264e-04, -1.377769e-02,-2.771053e-03, 8.241710e-04,-9.999958e-01 , -5.542117e-02,9.999644e-01, 7.969825e-03,
//-2.764397e-03, -2.918589e-01, 0., 0., 0., 1. ]
    R_rect_00.at<double>(0, 0) = 718.856;
    R_rect_00.at<double>(0, 1) = 0.0;
    R_rect_00.at<double>(0, 2) = 607.193;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = 0;
    R_rect_00.at<double>(1, 1) = 718.856;
    R_rect_00.at<double>(1, 2) = 185.216;
    R_rect_00.at<double>(1, 3) = 0;
    R_rect_00.at<double>(2, 0) = 0.0;
    R_rect_00.at<double>(2, 1) = 0.0;
    R_rect_00.at<double>(2, 2) = 1.0;
    R_rect_00.at<double>(2, 3) = 0;
 //   925.4797973632812  0.0  650.4112548828125
//0.0  925.6051025390625  360.46002197265625
//0.0  0.0  1.0
//tr
//data: [ 7.188560000000e+02, 0., 6.071928000000e+02 , 0.,7.188560000000e+02, 1.852157000000e+02, 0., 0., 1. ]
}

void projectLidarToCamera2(std::vector<Eigen::Vector3d> lidarPoints,pcl::PointCloud<pcl::PointXYZ> tempCloud,cv::Mat img) {
 
    // load Lidar points from file
  


    cv::Mat R_rect_00(3, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(R_rect_00, RT);


    // TODO: project lidar points
    cv::Mat visImg = img.clone();
    cv::Mat overlay = visImg.clone();

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
     pcl::PointCloud<pcl::PointXYZRGB> sumCloud1;
     
     int count =0;
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
        // filter the not needed points
        /*
        float MaxX = 25.0, maxY = 6.0, minZ = -1.40;
        if (it->x > MaxX ||it->x < 0.0 || abs(it->y) > maxY || it->z < minZ || it->r < 0.01) {
            continue;
        }*/

        // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
        X.at<double>(0, 0) = (*it)(0,0);
        X.at<double>(1, 0) = (*it)(1,0);
        X.at<double>(2, 0) = (*it)(2,0);
        X.at<double>(3, 0) = 1;
        double dis_sqr = X.at<double>(0, 0)*X.at<double>(0, 0)+X.at<double>(1, 0)*X.at<double>(1, 0)+X.at<double>(2, 0)*X.at<double>(2, 0);
        // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera. 
        // Store the result in Y.
        Y =  R_rect_00 * RT * X;
        // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
        if(pt.x>=0 && pt.x<1241 && pt.y >= 0 && pt.y <376 && X.at<double>(0, 0)>0.01&& dis_sqr<20*20 ){      //kitti 1241*376
          //  if(pt.x>=0 && pt.x<1280 && pt.y >= 0 && pt.y <720 && X.at<double>(0, 0)>0.01 ){
            pcl::PointXYZRGB pt_color;
            pt_color.x = tempCloud.points[count].x;
            pt_color.y = tempCloud.points[count].y;
            pt_color.z = tempCloud.points[count].z;
            pt_color.r = img.at<cv::Vec3b>(pt.y,pt.x)[2];
            pt_color.g = img.at<cv::Vec3b>(pt.y,pt.x)[1];
            pt_color.b = img.at<cv::Vec3b>(pt.y,pt.x)[0];

                sumCloud1.points.push_back(pt_color);
        float val =  (*it)(0,0);
        float maxVal = 20.0;
        int red = std::min(255, (int) (255 * abs((val - maxVal) / maxVal)));
        int green = std::min(255, (int) (255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1); // img  show in rviz
        }
        count++;
    }
    totalCloud+=sumCloud1;
    
    sensor_msgs::PointCloud2 test_cloud;
            pcl::toROSMsg(sumCloud1, test_cloud);

            test_cloud.header.frame_id = "/camera_init";
            pubcloud_test.publish(test_cloud);    // test_cloud 是添加了rgb信息的点云

//add tr
    sensor_msgs::PointCloud2 color_cloud;
            pcl::toROSMsg(totalCloud, color_cloud);

            color_cloud.header.frame_id = "/camera_init";
            pubcloud_all.publish(color_cloud);    // test_cloud 是添加了rgb信息的点云
    


    float opacity = 0.6;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg(); //图像转msg
    pubimg_test.publish(msg);
}











void CloudRegisteredHandler(
    const sensor_msgs::PointCloud2ConstPtr &CloudRegistered) {
  mBuf.lock();
 // std::cout <<4<<std::endl;
 //std::cout <<"CloudRegistered "<<CloudRegistered->header.stamp<<std::endl;
  CloudRegisteredBuf.push_back(CloudRegistered);
  mBuf.unlock();
}
void ImageHandler(
    const sensor_msgs::ImageConstPtr&Image) {
       // std::cout <<3<<std::endl;
    image_use_count++;
    if(image_use_count >=2){
        image_use_count = 0;
        //std::cout <<"iamge_now "<<Image->header.stamp.toSec()<<std::endl;
        mBuf.lock();
        ImageBuf.push_back(Image);
        mBuf.unlock();
    }
}
// receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  mBuf.lock();
  //std::cout <<"laserOdometry "<<laserOdometry->header.stamp.toSec()<<std::endl;
  //std::cout <<5<<std::endl;
  odometryBuf.push_back(laserOdometry);
  mBuf.unlock();
}




void process() {
    
    while (1) {
        std::cout <<" begin"<<" iamge "<<ImageBuf.size()<<" odom " <<odometryBuf.size()<<" clooud "<<CloudRegisteredBuf.size() <<" bool "<<(!ImageBuf.empty() && !odometryBuf.empty() && !CloudRegisteredBuf.empty() && ImageBuf.size()>5)<<std::endl;
        if((!ImageBuf.empty() )&& (!odometryBuf.empty()) && (!CloudRegisteredBuf.empty()) && (ImageBuf.size()>10))
        {
            //std::cout <<"begin"<<ImageBuf.size() <<odometryBuf.size()<<CloudRegisteredBuf.size() <<std::endl;
            std::cout <<" begin"<<" iamge "<<ImageBuf.size()<<" odom " <<odometryBuf.size()<<" clooud "<<CloudRegisteredBuf.size() <<" bool "<<(!ImageBuf.empty() && !odometryBuf.empty() && !CloudRegisteredBuf.empty() && ImageBuf.size()>5)<<std::endl;
            mBuf.lock();
            std::cout <<1<<std::endl;

            
            std::cout <<3<<std::endl;
            sensor_msgs::Image::ConstPtr imageData = ImageBuf.front();
            std::cout <<3.15<<std::endl;
            ImageBuf.pop_front();
            double image_timeCur = imageData->header.stamp.toSec();
            
            double odometry_timelast = 0;
            double odometry_timenext = 0;
            std::cout <<3.2<<std::endl;
      /*      
            while( !odometryBuf.empty() && image_timeCur>=odometryBuf.front()->header.stamp.toSec()){
                odometry_timelast = odometryBuf.front()->header.stamp.toSec();
                std::cout <<3.1<<"image"<<image_timeCur<<std::endl;
                std::cout <<3.3<<"last"<<odometry_timelast<<std::endl;
                q_wodom_last.x() = odometryBuf.front()->pose.pose.orientation.x;
                q_wodom_last.y() = odometryBuf.front()->pose.pose.orientation.y;
                q_wodom_last.z() = odometryBuf.front()->pose.pose.orientation.z;
                q_wodom_last.w() = odometryBuf.front()->pose.pose.orientation.w;
                t_wodom_last.x() = odometryBuf.front()->pose.pose.position.x;
                t_wodom_last.y() = odometryBuf.front()->pose.pose.position.y;
                t_wodom_last.z() = odometryBuf.front()->pose.pose.position.z;
                odometryBuf.pop_front();
            }
            
            
            std::cout <<3.3<<"next"<<odometryBuf.front()->header.stamp.toSec()<<std::endl;
            std::cout <<3.4<<"right "<< (image_timeCur > odometry_timelast) << "right "<<(image_timeCur <odometryBuf.front()->header.stamp.toSec())<<std::endl;
            std::cout <<4<<std::endl;
            
            odometry_timenext = odometryBuf.front()->header.stamp.toSec();
            q_wodom_next.x() = odometryBuf.front()->pose.pose.orientation.x;
            q_wodom_next.y() = odometryBuf.front()->pose.pose.orientation.y;
            q_wodom_next.z() = odometryBuf.front()->pose.pose.orientation.z;
            q_wodom_next.w() = odometryBuf.front()->pose.pose.orientation.w;
            t_wodom_next.x() = odometryBuf.front()->pose.pose.position.x;
            t_wodom_next.y() = odometryBuf.front()->pose.pose.position.y;
            t_wodom_next.z() = odometryBuf.front()->pose.pose.position.z;
            odometryBuf.pop_front();
 */  
            bool sign = false;
            for(Odoiter = odometryBuf.begin();
                        (Odoiter != odometryBuf.end()) && ( (image_timeCur)>(*Odoiter)->header.stamp.toSec());Odoiter++){
                sign =true;
            }
            if(sign == true){
                Odoiter--;
            }

                 odometry_timelast = (*Odoiter)->header.stamp.toSec();
                std::cout <<3.1<<"image"<<image_timeCur<<std::endl;
                std::cout <<3.3<<"last"<<odometry_timelast<<std::endl;
                q_wodom_last.x() = (*Odoiter)->pose.pose.orientation.x;
                q_wodom_last.y() = (*Odoiter)->pose.pose.orientation.y;
                q_wodom_last.z() = (*Odoiter)->pose.pose.orientation.z;
                q_wodom_last.w() = (*Odoiter)->pose.pose.orientation.w;
                t_wodom_last.x() = (*Odoiter)->pose.pose.position.x;
                t_wodom_last.y() = (*Odoiter)->pose.pose.position.y;
                t_wodom_last.z() = (*Odoiter)->pose.pose.position.z;        
            Odoiter++;
            odometry_timenext = (*(Odoiter))->header.stamp.toSec();
            q_wodom_next.x() = (*(Odoiter))->pose.pose.orientation.x;
            q_wodom_next.y() = (*(Odoiter))->pose.pose.orientation.y;
            q_wodom_next.z() = (*(Odoiter))->pose.pose.orientation.z;
            q_wodom_next.w() = (*(Odoiter))->pose.pose.orientation.w;
            t_wodom_next.x() = (*(Odoiter))->pose.pose.position.x;
            t_wodom_next.y() = (*(Odoiter))->pose.pose.position.y;
            t_wodom_next.z() = (*(Odoiter))->pose.pose.position.z;



            while(odometryBuf.front()->header.stamp.toSec()<odometry_timelast){
                odometryBuf.pop_front();
            }

            mBuf.unlock();
            double s;
            s = (image_timeCur-odometry_timelast)/(odometry_timenext-odometry_timelast);
            Eigen::Quaterniond q_wodom_cur = q_wodom_last.slerp(s, q_wodom_next);
            Eigen::Matrix3d r_wodom_cur(q_wodom_cur);
            Eigen::Vector3d t_wodom_cur = t_wodom_last+s * (t_wodom_next-t_wodom_last);
            Eigen::Matrix3d r_wodom_cur_inv = r_wodom_cur.inverse();
            Eigen::Vector3d t_wodom_cur_inv = -r_wodom_cur_inv*t_wodom_cur;
            
  

            std::cout <<"s "<<s<<std::endl;
            mBuf.lock();
            while(image_timeCur-0.1>CloudRegisteredBuf.front()->header.stamp.toSec()){
                CloudRegisteredBuf.pop_front();
            }

            std::cout <<"timep "<<CloudRegisteredBuf.front()->header.stamp.toSec()<<std::endl;
            std::cout <<"empty "<<CloudRegisteredBuf.empty()<<std::endl;
            sumCloud.clear();
            
            //while((!CloudRegisteredBuf.empty()) && ( (image_timeCur+0.4)>=(CloudRegisteredBuf.front()->header.stamp.toSec()))){
            for(Clouditer = CloudRegisteredBuf.begin();
                        ((Clouditer != CloudRegisteredBuf.end()) && ( (image_timeCur+0.1)>=((*Clouditer)->header.stamp.toSec())));Clouditer++){
                pcl::PointCloud<pcl::PointXYZ> tempCloud;
            
                pcl::fromROSMsg(*(*Clouditer), tempCloud);
                //std::cout << "temp_size"<<tempCloud.size()<<std::endl;
                sumCloud+=tempCloud;
                //CloudRegisteredBuf.pop_front();
            }
                std::cout <<"empty1 "<<CloudRegisteredBuf.empty()<<std::endl;
            mBuf.unlock();

            pc_temp.clear();
            for(int s =0;s<sumCloud.size();s++){
                Eigen::Vector3d pt(sumCloud.points[s].x,sumCloud.points[s].y,sumCloud.points[s].z);
                Eigen::Vector3d transform_pt = r_wodom_cur_inv*pt+t_wodom_cur_inv;
                pc_temp.push_back(transform_pt);
            }
            
            std::cout <<7<<std::endl;
            std::cout << "sum_size"<<sumCloud.size()<<std::endl;
            //test  这里的时间戳????

            /*   //delete by tr
            sensor_msgs::PointCloud2 test_cloud;
            pcl::toROSMsg(sumCloud, test_cloud);
            test_cloud.header.stamp = ros::Time().fromSec(image_timeCur); //点云的msg发布需要(1) header.stamp (2) header.frame_id (3) 点
            test_cloud.header.frame_id = "/camera_init";
*/



            //pubcloud_test.publish(test_cloud);
/*
            mBuf.lock();
            while(odometryBuf.size()>1000)
            {
                odometryBuf.pop_front();
            }
            std::cout <<2<<std::endl;
            while(CloudRegisteredBuf.size()>300)
            {
                CloudRegisteredBuf.pop_front();
            }
mBuf.unlock();
*/
            //当前新获取的图像保存在imageCur中
            try{
                cv_ptr =  cv_bridge::toCvCopy(imageData, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
            }
            catch(cv_bridge::Exception& e){  //异常处理
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat imageCur = cv_ptr->image;
/*

    Mat	cameraMatrix, distCoeffs, map1, map2;
	Size imageSize;
	char * IntrinsicsPath = "Intrinsics.yml";
	ReadIntrinsics(cameraMatrix, distCoeffs, imageSize, IntrinsicsPath);
 
	// 去畸变并保留最大图
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, map1, map2);
 
	Mat	imageCur_undis;
remap(imageCur, imageCur_undis, map1, map2, INTER_LINEAR);

*/
        projectLidarToCamera2(pc_temp,sumCloud,imageCur);

            
            
        }
        std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
    }
    
}


bool savemap(std_srvs::Trigger::Request  &req,std_srvs::Trigger::Response &res)
{
  pcl::io::savePCDFileASCII ("/home/tr/catkin_ws/test_pcd.pcd", totalCloud);
  res.success = true;
  res.message = "true";
  //res.string = "true";
  std::cout <<"map_save_complete"<<std::endl;
  return true;
}



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "map_building");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("/savemap", savemap);
    ros::Subscriber subCloudRegistered = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 10000,CloudRegisteredHandler);
    std::cout <<1<<std::endl;
    //ros::Subscriber subImage =nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 100,ImageHandler);
    ros::Subscriber subImage =nh.subscribe<sensor_msgs::Image>("/kitti/camera_color_left/image_raw", 10000,ImageHandler);

    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 10000, laserOdometryHandler);

    //pubColorMap = nh.advertise<sensor_msgs::PointCloud2>("/color_map", 100);

    std::cout <<2<<std::endl;
    pubcloud_test = nh.advertise<sensor_msgs::PointCloud2>("/sumcloud_test", 100); //sumcloud_test 是加rgb信息的点云
    pubcloud_all  = nh.advertise<sensor_msgs::PointCloud2>("/sumcloud_color", 100);
    pubimg_test = nh.advertise<sensor_msgs::Image>("/sumimg_test", 100);//sumimg是添加了激光点投影到图像的img
    std::cout <<2.1<<std::endl;

    std::thread mapping_process{process};
    std::cout <<2.2<<std::endl;
    ros::spin();

    return 0;
}
