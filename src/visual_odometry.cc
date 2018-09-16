/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 22:41:04
  * @last_modified_date: 2018-09-16 23:39:13
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/visual_odometry.hh>

//CODE
namespace myslam
{
  VisualOdometry::VisualOdometry(int number)
    : VisualOdometry()
  {
    p_orb = cv::ORB::create(number);
  }

  int VisualOdometry::addFrame(myslam::Frame::Ptr& frame_will_set)
  {
    auto image = frame_will_set->getImage();
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    this->detectAndCompute(image,
                           keypoints,
                           descriptors);
    frame_will_set->setKeypoints(keypoints);
    frame_will_set->setDescriptors(descriptors);
    frames_.push_back(frame_will_set);
    return 0;
  }

  int VisualOdometry::detectAndCompute(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints,
                                      cv::Mat& descriptors)
  {
    p_orb->detect(image, keypoints);
    p_orb->compute(image, keypoints, descriptors);
    return 0;
  }

}
