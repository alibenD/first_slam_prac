#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:33:08
  * @last_modified_date: 2018-08-11 23:23:20
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
      typedef std::shared_ptr<Frame> Ptr;   /*!< A datatype shared_pointer points Frame */

      Frame() = default;

      /**
       * @brief A constructor of Frame with camer and TF info.
       * @param[in] id Identification to assign to this new frame
       * @param[in] timestamp The timestamp of this frame
       * @param[in] T_camera_world A transformation from world to camera(Lie Algebra)
       * @param[in] color The channels of BGR info in this frame
       * @param[in] depth The channel of depth info in this frame
       */
      Frame(long id,
            double timestamp=0,
            Sophus::SE3<double> T_camera_world=Sophus::SE3<double>(),
            Camera::Ptr camera=nullptr,
            cv::Mat color=cv::Mat(),
            cv::Mat depth=cv::Mat());


      virtual ~Frame() = default;

      /**
       * @brief Create a new frame(on heap memory) and return a shared_pointer
       * @return a shared_pointer pointed this new instance of Frame
       */
      static Frame::Ptr createFrame();

      /**
       * @brief Given a key_point, estimate the depth of this key_point
       * @param[in] key_point A keypoint to be estimated depth
       * @return Depth of this key_point
       */
      double findDepth(const cv::KeyPoint& key_point);

      /**
       * @brief A method to get camera center in the world frame
       * @return A coordinate of center of camera in the world frame
       */
      Eigen::Vector3d getCameraCenter() const;

      /**
       * @brief Check if a point in the world frame is in this frame
       * @param[in] point_world A coordinate of point in the world frame
       * @retval true: Yes
       * @retval false: No
       */
      bool isInFrame(const Eigen::Vector3d& point_world);

      // Get methods
      /**
       * @brief Get the identification of this frame
       * @return id of this frame
       */
      inline unsigned long get_id() const
      { return id_;}

      /**
       * @brief Get the timestamp of this frame
       * @return timestamp of this frame
       */
      inline double get_timestamp() const
      { return timestamp_;}

      /**
       * @brief Get the TF from world to camera coordinate
       */
      inline const Sophus::SE3<double>& get_Tcw()
      { return T_camera_world_; }

      /**
       * @brief Get the color channels of this frame
       * @param[out] color_will_get color channels of this frame
       */
      inline int get_color(cv::Mat& color_will_get) const
      {
        color_will_get = color_;
        return 0;
      }

      /**
       * @brief Get the color channels of this frame
       * @return A reference of the channels of BGR colors
       */
      inline const cv::Mat& get_color() const
      {
        return color_;
      }

      /**
       * @brief Get the depth channel of this frame
       * @param[out] depth_will_get depth channel of this frame
       */
      inline int get_depth(cv::Mat& depth_will_get) const
      {
        depth_will_get = depth_;
        return 0;
      }

      // Set methods
      /**
       * @brief Set an identification for this frame
       * @param[in] id_will_set new id for this frame
       */
      inline int set_id(unsigned long id_will_set)
      {
        id_ = id_will_set;
        return 0;
      }

      /**
       * @brief Set an timestamp for this frame
       * @param[in] timestamp_will_set new timestamp for this frame
       */
      inline int set_timestamp(double timestamp_will_set)
      {
        timestamp_ = timestamp_will_set;
        return 0;
      }

      /**
       * @brief Set a new TF from world to camera
       * @param[in] Tcw_will_set A new TF
       */
      inline int set_Tcw(const Sophus::SE3<double>& Tcw_will_set)
      {
        T_camera_world_ = Tcw_will_set;
        return 0;
      }

      /**
       * @brief Set an color for this frame
       * @param[in] color_will_set new color for this frame
       */
      inline int set_color(const cv::Mat& color_will_set)
      {
        color_ = color_will_set.clone();
        return 0;
      }

      /**
       * @brief Set an depth for this frame
       * @param[in] depth_will_set new depth for this frame
       */
      inline int set_depth(const cv::Mat& depth_will_set)
      {
        depth_ = depth_will_set;
        return 0;
      }


    public:
      Camera::Ptr camera_;    /*!< A datatype shared_pointer point Camera */

    private:
      unsigned long id_;      /*!< The identification of this frame */
      double timestamp_;      /*!< The time when the frame was recorded */
      Sophus::SE3<double> T_camera_world_; /*!< A transformation from world to camera */
      cv::Mat color_;         /*!< The BGR channels of this frame */
      cv::Mat depth_;         /*!< The depth_ channel of this frame */
  };

}   // END of namespace myslam

#endif // __FRAME_HH__
