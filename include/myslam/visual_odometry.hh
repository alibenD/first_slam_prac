#ifndef __VISUAL_ODOMETRY_HH__
#define __VISUAL_ODOMETRY_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: visual_odometry.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-02 08:35:52
  * @last_modified_date: 2018-08-16 12:30:51
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>
#include <myslam/map.hh>
#include <opencv2/features2d/features2d.hpp>

// Declaration
namespace myslam
{
  class VisualOdometry
  {

    public:
      typedef std::shared_ptr<VisualOdometry> Ptr;  /**< A share_pointer points VisualOdometry*/
      enum VOStatus
      {
        INITIALIZING = -1,
        OK = 0,
        LOST
      };
      typedef enum VOStatus eVOStatus;

    private:
      VOStatus status_;               /**< VO status */
      Map::Ptr map_;                  /**< A map pointer */
      Frame::Ptr pFrame_reference_;    /**< A reference frame pointer */
      Frame::Ptr pFrame_current_;      /**< A current frame pointer */

      cv::Ptr<cv::ORB> pOrb_;          /**< A detector/computer of ORB */
      std::vector<cv::Point3f> point_ref_frame_;        /**< 3D points in reference frame */
      std::vector<cv::KeyPoint> keypoints_curr_frame_;  /**< Keypoints in   */
      cv::Mat descriptors_curr_;      /**< Descriptor in current frame */
      cv::Mat descriptors_ref_;       /**< Descriptor in reference frame */
      std::vector<cv::DMatch> feature_matches_;   /**< Features matched */

      Sophus::SE3<double> T_curr_ref_estimated_;    /**< Estimation TF from ref to curr */
      int num_inliers_;   /**< The number of inlier features in ICP */
      int num_lost_;      /**< The number of lose times */


      int num_of_features_;   /**< The number of features */
      double scale_factor_;   /**< The scale value of image pyramid */
      int level_pyramid_;     /**< The number of pyramid levels */
      float match_ratio_;     /**< The ratio for selecting good matched */
      int max_num_lost_;      /**< The max number of continuous lost times */
      int min_inliers_;       /**< The minimum number of inliers */

      double keyframe_min_rot_;     /**< The minimal rotation of two key frames */
      double keyframe_min_trans_;   /**< The minimal transformation of two key frames*/

    public:
      VisualOdometry();
      virtual ~VisualOdometry() = default;

      /**
       * @brief Add a new frame and check if it is a key frame
       * @param[in] frame A new frame from camera
       */
      bool addFrame(Frame::Ptr frame);

      /**
       * @brief Get the status of this VO
       */
      eVOStatus getStatus()
      { return status_; }

      /**
       * @brief Set the estimated transfomation from reference to current frame
       */
      int setTransformationEstimation(const Sophus::SE3<double> T_c_r_will_set)
      {
        T_curr_ref_estimated_ = T_c_r_will_set;
        return 0;
      }

    protected:
      int extractKeyPoints();
      int computeDescriptors();
      int featureMatching();
      int poseEstimationPnP();
      int setRef3DPoints();

      int addKeyFrame();
      bool checkEstimatePose();
      bool checkKeyFrame();
  };
}   // END of namespace myslam
#endif // __VISUAL_ODOMETRY_HH__
