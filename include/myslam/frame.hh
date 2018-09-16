#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 11:29:19
  * @last_modified_date: 2018-09-16 23:35:47
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <fstream>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// Declaration
namespace myslam
{
  class Frame;

  class Frame
  {
    public:
      Frame() = default;
      Frame(long long id);
      ~Frame() = default;
      static long long number_;

    public:
      using Ptr = std::shared_ptr<Frame>;
      static Frame::Ptr createFrame(const cv::Mat& image);

      inline const cv::Mat& getImage() const
      { return image_; }
      inline int setImage(const cv::Mat& image)
      {
        image_ = image.clone();
        return 0;
      }

      inline const std::vector<cv::KeyPoint>& getKeypoints() const
      { return keypoints_; }
      inline int setKeypoints(const std::vector<cv::KeyPoint>& keypoint_will_set)
      {
        keypoints_ = keypoint_will_set;
        return 0;
      }

      inline int setDescriptors(const cv::Mat& descriptors_will_set)
      {
        descriptors_ = descriptors_will_set.clone();
        return 0;
      }

    private:
      long long id_;
      cv::Mat image_;
      std::vector<cv::KeyPoint> keypoints_;
      std::vector<cv::Point3f> spacial_points_;
      cv::Mat descriptors_;
  };
}   // END of namespace Frame
#endif // __FRAME_HH__
