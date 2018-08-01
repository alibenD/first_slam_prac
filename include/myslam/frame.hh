#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:33:08
  * @last_modified_date: 2018-08-01 16:57:56
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>
#include <myslam/camera.hh>

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
      typedef std::shared_ptr<Frame> Ptr;
      Frame() = default;
      Frame(long id,
            double time_stamp=0,
            Sophus::SE3<double> T_camera_world=Sophus::SE3<double>(),
            Camera::Ptr camera=nullptr,
            cv::Mat color=cv::Mat(),
            cv::Mat depth=cv::Mat());
      virtual ~Frame() = default;

      static Frame::Ptr createFrame();
      double findDepth(const cv::KeyPoint& key_point);
      Eigen::Vector3d getCameraCenter() const;
      bool isInFrame(const Eigen::Vector3d& point_world);

      // Get methods
      inline unsigned long get_id() const
      { return id_;}

      inline double get_timestamp() const
      { return timestamp_;}

      inline int get_color(cv::Mat& color_will_get) const
      {
        color_will_get = color_;
        return 0;
      }
      inline int get_depth(cv::Mat& depth_will_get) const
      {
        depth_will_get = depth_;
        return 0;
      }

      // Set methods
      inline int set_id(unsigned long id_will_set)
      {
        id_ = id_will_set;
        return 0;
      }

      inline int set_timestamp(double timestamp_will_set)
      {
        timestamp_ = timestamp_will_set;
        return 0;
      }

      inline int set_color(const cv::Mat& color_will_set)
      {
        color_ = color_will_set;
        return 0;
      }

      inline int set_depth(const cv::Mat& depth_will_set)
      {
        depth_ = depth_will_set;
        return 0;
      }


    public:
      Camera::Ptr camera_;

    private:
      unsigned long id_;      /*!< The identification of this frame */
      double timestamp_;     /*!< The time when the frame was recorded */
      Sophus::SE3<double> T_camera_world_; /*!< A transformation from world to camera */
      cv::Mat color_;         /*!< The BGR channels of this frame */
      cv::Mat depth_;         /*!< The depth_ channel of this frame */
  };

}   // END of namespace myslam

#endif // __FRAME_HH__
