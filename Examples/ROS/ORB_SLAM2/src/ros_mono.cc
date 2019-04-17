/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <chrono>
using namespace std::chrono;
using namespace tf;
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    geometry_msgs::Pose GetPose();
    ORB_SLAM2::System* mpSLAM;
    
    geometry_msgs::Pose camera_pose;
    std::mutex pose_mutex;

    geometry_msgs::Pose marker_pose[4];
    std::mutex marker_mutex;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pos_pub = nodeHandler.advertise<geometry_msgs::Pose>("/slam/pos", 1);
    ros::Publisher pos_pub1 = nodeHandler.advertise<geometry_msgs::Pose>("/slam/marker1", 1);
    ros::Publisher pos_pub2 = nodeHandler.advertise<geometry_msgs::Pose>("/slam/marker2", 1);
    ros::Publisher pos_pub3 = nodeHandler.advertise<geometry_msgs::Pose>("/slam/marker3", 1);
    ros::Publisher pos_pub4 = nodeHandler.advertise<geometry_msgs::Pose>("/slam/marker4", 1);
    
    // ros::Rate loop_rate(0.1);
    chrono::time_point<chrono::system_clock> start, current;
    start = chrono::system_clock::now();
    while (ros::ok()) {
        current = chrono :: system_clock::now();
        if (current-start> std::chrono::milliseconds(100)) {
             start = current;
            pos_pub.publish(igb.GetPose());
            pos_pub1.publish(igb.marker_pose[0]);
            pos_pub2.publish(igb.marker_pose[1]);
            pos_pub3.publish(igb.marker_pose[2]);
            pos_pub4.publish(igb.marker_pose[3]);
        }
      ros::spinOnce();
    //   loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (pose.empty()) {
	    return;
    }
    // transform into right handed camera frame
    tf::Matrix3x3 rh_cameraPose(  - pose.at<float>(0,0),   pose.at<float>(0,1),   pose.at<float>(0,2),
                                  - pose.at<float>(1,0),   pose.at<float>(1,1),   pose.at<float>(1,2),
                                    pose.at<float>(2,0), - pose.at<float>(2,1), - pose.at<float>(2,2));

    tf::Vector3 rh_cameraTranslation( pose.at<float>(0,3),pose.at<float>(1,3), - pose.at<float>(2,3) );

    //rotate 270deg about z and 270deg about x
    tf::Matrix3x3 rotation270degZX( 0, 0, 1,
                                   -1, 0, 0,
                                    0,-1, 0);

    //publish right handed, x forward, y right, z down (NED)
//  static tf::TransformBroadcaster br;
//  tf::Transform transformCoordSystem = tf::Transform(rotation270degZX,tf::Vector3(0.0, 0.0, 0.0));
//  br.sendTransform(tf::StampedTransform(transformCoordSystem, ros::Time::now(), "camera_link", "camera_pose"));

//  tf::Transform transformCamera = tf::Transform(rh_cameraPose,rh_cameraTranslation);
//  br.sendTransform(tf::StampedTransform(transformCamera, ros::Time::now(), "camera_pose", "pose"));
    geometry_msgs::Pose p;
    p.position.x = rh_cameraTranslation[0];
    p.position.y = rh_cameraTranslation[1];
    p.position.z = rh_cameraTranslation[2];
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;

    {
        unique_lock<mutex> lock(pose_mutex);
        camera_pose = p;
    }
    {
        unique_lock<mutex> lock(marker_mutex);
        float (*pc)[3] = (float (*)[3])mpSLAM->getPC();
  
        marker_pose[0].position.x = pc[0][0];   
        marker_pose[0].position.y = pc[0][1];
        marker_pose[0].position.z = pc[0][2];

        marker_pose[1].position.x = pc[1][0];
        marker_pose[1].position.y = pc[1][1];
        marker_pose[1].position.z = pc[1][2];

        marker_pose[2].position.x = pc[2][0];
        marker_pose[2].position.y = pc[2][1];
        marker_pose[2].position.z = pc[2][2];

        marker_pose[3].position.x = pc[3][0];
        marker_pose[3].position.y = pc[3][1];
        marker_pose[3].position.z = pc[3][2];
        delete [] pc;
    }
}
geometry_msgs::Pose ImageGrabber::GetPose()
{
    unique_lock<mutex> lock(pose_mutex);
    return camera_pose;
}


