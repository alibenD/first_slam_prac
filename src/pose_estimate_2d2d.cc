/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: pose_estimate_2d2d.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-11 09:15:01
  * @last_modified_date: 2018-09-11 16:04:17
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/pose_estimate_2d2d.hh>

//CODE

void pose_estimate(const std::vector<cv::KeyPoint>& keypoint_last,
                   const std::vector<cv::KeyPoint>& keypoint_current,
                   const std::vector<cv::DMatch>& matches,
                   const cv::Mat& intrinsic,
                   cv::Mat& rotation,
                   cv::Mat& t)
{
  std::vector<cv::Point2f> point_last;
  std::vector<cv::Point2f> point_current;

  for(int i=0; i<(int)matches.size(); i++)
  {
    point_last.push_back(keypoint_last[matches[i].queryIdx].pt);
    point_current.push_back(keypoint_current[matches[i].trainIdx].pt);
  }

  cv::Mat fundamental_matrix;
  cv::Mat essential_matrix;
  cv::Mat homography_matrix;

  fundamental_matrix = cv::findFundamentalMat(point_last, point_current, cv::FM_RANSAC);
  
  auto focal_length = intrinsic.at<double>(1,1);
  auto principal = cv::Point2f(intrinsic.at<double>(0,2),
                               intrinsic.at<double>(1,2));

  essential_matrix = cv::findEssentialMat(point_last,
                                          point_current,
                                          focal_length,
                                          principal,
                                          cv::FM_8POINT);
                                          //cv::RANSAC);
  cv::recoverPose(essential_matrix, point_last, point_current, rotation, t, focal_length, principal);
}
