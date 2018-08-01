#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:33:08
  * @last_modified_date: 2018-08-01 09:56:07
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>

/**
 * @defgroup DataType
 * @{
 */

/**
 * @}
 */

// Declaration
namespace myslam
{
  /**
   * @ingroup DataType
   * @brief A class restoring frame from camera
   */
  class Frame
  {
    public:
      Frame() = default;
      virtual Frame() = default;
      static Frame::Ptr createFrame();
      double findDepth(const cv::KeyPoint& key_point);
      Eigen::Vector3d getCameraCenter() cosnt;
      bool isInFrame(const Vector3d& point_world);


    public:
      typedef std::shared_ptr<Frame> Ptr;
      Camera::Ptr camera_;

    private:
      unsigned long id_;      /*!< The identification of this frame */
      double time_stamp_;     /*!< The time when the frame was recorded */
      Sophus::SE3 T_camera_world; /*!< A transformation from world to camera */
      cv::Mat color_;         /*!< The BGR channels of this frame */
      cv::Mat depth_;         /*!< The depth_ channel of this frame */
  };

}   // END of namespace myslam

#endif // __FRAME_HH__
