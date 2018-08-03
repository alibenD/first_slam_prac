/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-02 10:35:36
  * @last_modified_date: 2018-08-03 09:03:56
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_odometry.hh>
#include <myslam.config.h>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>


//CODE
namespace myslam
{
  VisualOdometry::VisualOdometry()
    : status_(INITIALIZING),
      reference_frame_(nullptr),
      current_frame_(nullptr),
      map_(new Map),
      num_lost_(0),
      num_inliers_(0)
  {
    num_of_features_    = Config::get<int> ("number_of_features");
    scale_factor_       = Config::get<double> ("scale_factor");
    level_pyramid_      = Config::get<int> ("level_pyramid");
    match_ratio_        = Config::get<float> ("match_ratio");
    max_num_lost_       = Config::get<float> ("max_num_lost");
    min_inliers_        = Config::get<int> ("min_inliers");
    keyframe_min_rot_   = Config::get<double> ("keyframe_rotation");
    keyframe_min_trans_ = Config::get<double> ("keyframe_transformation");
    orb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
  }

  bool VisualOdometry::addFrame(Frame::Ptr frame)
  {
    switch( state_ )
    {
      case INITIALIZING:
        {
          status_ = OK;
          current_frame_ = reference_frame_ = frame;
          map_->insertKeyFrame(frame);
          extractKeyPoints();
          computeDescriptors();
          setRef3DPoints();
          break;
        }
      case OK:
        {
          current_frame_ = frame;
          extractKeyPoints();
          computeDescriptors();
          featureMatching();
          poseEstimationPnP();
          if( checkEstimatePose() == true )
          {
            current_frame_->T_camera_world_ = T_curr_ref_estimated_ * reference_frame_->T_camera_world_;
            reference_frame_ = current_frame_;
            setRef3DPoints();
            num_lost_ = 0;
            if( checkKeyFrame() == true )
            {
              addKeyFrame();
            }
          }
          else
          {
            num_lost_++;
            if(num_lost_ > max_num_lost_)
            {
              status_ = LOST;
            }
            return false;
          }
          break;
        }
      case LOST:
        {
          std::cout << "VO has lost." << std::endl;
          break;
        }
    }
  }

  int VisualOdometry::extractKeyPoints()
  {
    orb_->detect( current_frame_->color_, keypoints_curr_frame_ );
    return 0;
  }

  int VisualOdometry::computeDescriptors()
  {
    orb_->compute( current_frame_->color_, keypoints_curr_frame_, descriptors_curr_ );
    return 0;
  }

  int VisualOdometry::featureMatching()
  {
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors_ref_, descriptors_curr_, matches);
    float min_distance = std::min_element(matches.begin(),
                                          matched.end(),
                                          [](const cv::DMatch& m1, const cv::DMatch& m2)
    {
      return m1.distance < m2.distance;
    })->distance;
    feature_matches_.clear();
    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_distance*match_ratio_, 30.0))
      {
        feature_matches_.push_back(m);
      }
    }
    std::cout << "Good matches: " << feature_matches_.size() << std::endl;
    return 0;
  }

  int VisualOdometry::setRef3DPoints()
  {
    point_ref_frame_.clear();
    descriptors_ref_ = cv::Mat();
    for(size_t i=0; i<keypoints_curr_frame_.size();i++)
    {
      double depth = reference_frame_->findDepth(keypoints_curr_[i]);
      if(depth > 0)
      {

        cv::Vector3d point_camera = reference_frame_->camera_->pixel2camer(cv::Vector2d(keypoints_curr_frame_[i].pt.x,keypoints_curr_frame_[i].pt.y), depth);
        point_ref_frame_.push_back(cv::Point3f(point_camera(0,0),
                                               point_camera(1,0),
                                               point_camera(2.0)));
        descriptors_ref_.push_back(descriptors_curr_.row(i));
      }
    }
  }
  
  int VisualOdometry::poseEstimationPnP()
  {
    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point2f> points_2d;
    for(cv::DMatach m:feature_matches_)
    {
      points_3d.push_back(point_ref_frame_[m.queryIdex]);
      points_3d.push_back(keypoints_curr_)
    }

    float fx, fy, cx, cy;
    fx = reference_frame_->camera_->get_fx();
    fy = reference_frame_->camera_->get_fy();
    cx = reference_frame_->camera_->get_cx();
    cy = reference_frame_->camera_->get_cy();
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                       fx, 0, cx,
                       0, fy, cy,
                       0,  1,  0);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(points_3d, points_2d, K,
                       cv::Mat(), rev, tvec,
                       false, 100, 4.0,
                       0.99, inliers);
    num_inliers_ = inliers.rows;
    std::cout << "PnP inliers: " << num_inliers_ << std::endl;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv::cv2eigen(R, r);

    Eigen::AngleAxisd angle(r);
    Eigen::Quateriniond q(r);
    T_curr_ref_estimated_ = Sophus::SE3<double>(Sophus::SO3<double>(q), Eigen::Vector3d(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)));
    return 0;
  }

  bool VisualOdometry::checkEstimatePose()
  {
    if(num_inliers_ < min_inliers_)
    {
      std::cout << "Reject because inlier is too few: " << num_inliers_ << std::endl;
      return false;
    }
    Sophus::Vector6d d = T_curr_ref_estimated_.log();
    if(d.norm() > 5.0)
    {
      std::cout << "Reject because motion is too large: " << d.norm() << std::endl;
      return false;
    }
    return true;
  }

  bool VisualOdometry::checkKeyFrame()
  {
    Sophus::Vector6d d = T_curr_ref_estimated_.log();
    Sophus::Vector3d transformation = d.head<3>();
    Sophus::Vector3d rotation       = d.tail<3>();
    if( rotation.norm() > keyframe_min_rot_ ||
        transformation.norm() > keyframe_min_trans_)
    {
      return true;
    }
    return false;
  }

  int VisualOdometry::addKeyFrame()
  {
    std::cout << "Adding a key-frame." << std::endl;
    map_->insertKeyFrame(current_frame_);
  }
} // END of namespace myslam
