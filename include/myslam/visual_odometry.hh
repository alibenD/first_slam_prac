#ifndef __VISUAL_ODOMETRY_HH__
#define __VISUAL_ODOMETRY_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: visual_odometry.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 11:05:16
  * @last_modified_date: 2018-09-16 23:39:28
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <myslam/frame.hh>

// Declaration
namespace myslam
{
  class VisualOdometry;

  class VisualOdometry
  {
    public:
      VisualOdometry() = default;
      VisualOdometry(int number);
      ~VisualOdometry() = default;

    public:
      inline int setK(const cv::Mat& K_will_set)
      {
        K_ = K_will_set.clone();
        return 0;
      }
      inline const std::vector<myslam::Frame::Ptr> getFrames() const
      {
        return frames_;
      }

      int addFrame(myslam::Frame::Ptr& frame_will_set);
      int detectAndCompute(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors);

    private:
      cv::Mat K_;
      std::vector<myslam::Frame::Ptr> frames_;
      cv::Ptr<cv::ORB> p_orb;
  };
  ;
}   // END of namespace myslam
#endif // __VISUAL_ODOMETRY_HH__
