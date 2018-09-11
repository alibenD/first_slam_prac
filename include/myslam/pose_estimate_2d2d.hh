#ifndef __POSE_ESTIMATE_2D2D_HH__
#define __POSE_ESTIMATE_2D2D_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: pose_estimate_2d2d.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-11 10:22:44
  * @last_modified_date: 2018-09-11 10:44:33
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Declaration
void pose_estimate(const std::vector<cv::KeyPoint>& keypoint_last,
                   const std::vector<cv::KeyPoint>& keypoint_current,
                   const std::vector<cv::DMatch>& matches,
                   const cv::Mat& intrinsic,
                   cv::Mat& Rotation,
                   cv::Mat& t);

#endif // __POSE_ESTIMATE_2D2D_HH__
