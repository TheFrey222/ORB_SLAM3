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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>
#include<opencv2/sfm/projection.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);
  
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

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }

      mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


