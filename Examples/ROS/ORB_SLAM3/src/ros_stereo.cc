/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include<opencv2/sfm/projection.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        int rows = fsSettings["Camera.height"];
        int cols = fsSettings["Camera.width"];

        cv::Mat K_l(3, 3, CV_32F);
        cv::Mat K_r(3, 3, CV_32F);
        cv::Mat D_l(1, 4, CV_32F);
        cv::Mat D_r(1, 4, CV_32F); 
        cv::Mat R_r, P_r, cvTlr, t_r;

        K_l.at<float>(0,0) = fsSettings["Camera1.fx"];
        K_l.at<float>(1,1) = fsSettings["Camera1.fy"];
        K_l.at<float>(0,2) = fsSettings["Camera1.cx"];
        K_l.at<float>(1,2) = fsSettings["Camera1.cy"];
        K_l.at<float>(0,1) = 0.0;
        K_l.at<float>(1,0) = 0.0;
        K_l.at<float>(2,0) = 0.0;
        K_l.at<float>(2,1) = 0.0;
        K_l.at<float>(2,2) = 1.0;

        cout << "K_l = " << endl << " " << K_l << endl << endl;

        K_r.at<float>(0,0) = fsSettings["Camera2.fx"];
        K_r.at<float>(1,1) = fsSettings["Camera2.fy"];
        K_r.at<float>(0,2) = fsSettings["Camera2.cx"];
        K_r.at<float>(1,2) = fsSettings["Camera2.cy"];
        K_r.at<float>(0,1) = 0.0;
        K_r.at<float>(1,0) = 0.0;
        K_r.at<float>(2,0) = 0.0;
        K_r.at<float>(2,1) = 0.0;
        K_r.at<float>(2,2) = 1.0;

        cout << "K_r = " << endl << " " << K_r << endl << endl;

        D_l.at<float>(0,0) = fsSettings["Camera1.k1"];
        D_l.at<float>(0,1) = fsSettings["Camera1.k2"];
        D_l.at<float>(0,2) = fsSettings["Camera1.p1"];
        D_l.at<float>(0,3) = fsSettings["Camera1.p2"];

        cout << "D_l = " << endl << " " << D_l << endl << endl;

        D_r.at<float>(0,0) = fsSettings["Camera2.k1"];
        D_r.at<float>(0,1) = fsSettings["Camera2.k2"];
        D_r.at<float>(0,2) = fsSettings["Camera2.p1"];
        D_r.at<float>(0,3) = fsSettings["Camera2.p2"];

        cout << "D_r = " << endl << " " << D_r << endl << endl;

        cv::Rect roi_l, roi_r;
        
        cvTlr = fsSettings["Stereo.T_c1_c2"].mat();
        R_r = cvTlr.rowRange(0,3).colRange(0,3);
        t_r = cvTlr.rowRange(0,3).colRange(3,4);

        cout << "cvTlr = " << endl << " " << cvTlr << endl << endl;
        cout << "R_r = " << endl << " " << R_r << endl << endl;
        cout << "t_r = " << endl << " " << t_r << endl << endl;

        cv::sfm::projectionFromKRt(K_r, R_r, t_r, P_r);

        cout << "P_r = " << endl << " " << P_r << endl << endl;


        if(K_l.empty())
        {
            cerr << "ERROR: Calibration parameters K_l to rectify stereo are missing!" << endl;
            return -1;
        }
        else if(K_r.empty())
        {
            cerr << "ERROR: Calibration parameters K_r to rectify stereo are missing!" << endl;
            return -1;
        } 
        else if(P_r.empty())
        {
            cerr << "ERROR: Calibration parameters P_r to rectify stereo are missing!" << endl;
            return -1;
        }
        else if(R_r.empty())
        {
            cerr << "ERROR: Calibration parameters R_r to rectify stereo are missing!" << endl;
            return -1;
        }
        else if(D_l.empty())
        {
            cerr << "ERROR: Calibration parameters D_l to rectify stereo are missing!" << endl;
            return -1;
        }
        else if(D_r.empty())
        {
            cerr << "ERROR: Calibration parameters D_r to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, cv::Mat(), 
                                    cv::getOptimalNewCameraMatrix(K_l, D_l, cv::Size(cols,rows), 1, cv::Size(cols,rows), 0),
                                    cv::Size(cols,rows), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r, cv::Size(cols,rows), CV_32F, igb.M1r, igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}


