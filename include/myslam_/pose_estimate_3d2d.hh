#ifndef __POSE_ESTIMATE_3D2D_HH__
#define __POSE_ESTIMATE_3D2D_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: pose_estimate_3d2d.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-11 22:52:55
  * @last_modified_date: 2018-09-14 09:16:54
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Declaration
void pose_estimate_3d2d(const std::vector<cv::KeyPoint>& keypoint_last,
                        const std::vector<cv::KeyPoint>& keypoint_current,
                        const std::vector<cv::Point3f>& spatial_point,
                        const std::vector<cv::DMatch>& matches,
                        const cv::Mat& intrinsic,
                        cv::Mat& rotation,
                        cv::Mat& t);

void bundleAdjustment(const std::vector<cv::Point3f> point3d,
                      const std::vector<cv::Point2f> point2d,
                      const cv::Mat& intrinsic,
                      cv::Mat& R,
                      cv::Mat& t);
void bundleAdjustmentDepre(const std::vector<cv::Point3f>& point3d,
                      const std::vector<cv::Point2f>& point2d,
                      const cv::Mat& intrinsic,
                      const cv::Mat& inliers,
                      cv::Mat& R,
                      cv::Mat& t);
#endif // __POSE_ESTIMATE_3D2D_HH__
