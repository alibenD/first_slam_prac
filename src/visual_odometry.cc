/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-02 10:35:36
  * @last_modified_date: 2018-08-16 11:46:19
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/visual_odometry.hh>
#include <myslam/config.hh>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>


//CODE
namespace myslam
{
  VisualOdometry::VisualOdometry()
    : status_(INITIALIZING),
      pFrame_reference_(nullptr),
      pFrame_current_(nullptr),
      map_(std::make_shared<Map>()),
      T_curr_ref_estimated_(),
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
    pOrb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
  }

  bool VisualOdometry::addFrame(Frame::Ptr pFrame)
  {
    switch( status_ )
    {
      case INITIALIZING:
        {
          status_ = OK;
          pFrame_current_ = pFrame_reference_ = pFrame;
          map_->insertKeyFrame(pFrame);
          extractKeyPoints();
          computeDescriptors();
          setRef3DPoints();
          break;
        }
      case OK:
        {
          pFrame_current_ = pFrame;
          extractKeyPoints();
          computeDescriptors();
          featureMatching();
          poseEstimationPnP();
          if( checkEstimatePose() == true )
          {
            //std::cout << "[Bebug] Before get_tcw" << std::endl;
            //auto Tcw_reference = pFrame_reference_->get_Tcw();
            //std::cout << "[Bebug] After get_tcw" << std::endl;
            //auto Tcw_current = T_curr_ref_estimated_;

            //std::cout << "[Bebug] Before set_tcw" << std::endl;
            //pFrame_current_->set_Tcw(Tcw_current);
            pFrame_current_->set_Tcw(T_curr_ref_estimated_ * pFrame_reference_->get_Tcw());
            pFrame_reference_ = pFrame;
            //std::cout << "[Bebug] After set_tcw" << std::endl;
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
      default:
        break;
    }
    return true;
  }

  int VisualOdometry::extractKeyPoints()
  {
    pOrb_->detect( pFrame_current_->get_color(), keypoints_curr_frame_ );
    return 0;
  }

  int VisualOdometry::computeDescriptors()
  {
    pOrb_->compute( pFrame_current_->get_color(), keypoints_curr_frame_, descriptors_curr_ );
    return 0;
  }

  int VisualOdometry::featureMatching()
  {
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors_ref_, descriptors_curr_, matches);
    float min_distance = std::min_element(matches.begin(),
                                          matches.end(),
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
      double depth = pFrame_reference_->findDepth(keypoints_curr_frame_[i]);
      if(depth > 0)
      {

        Sophus::Vector3d point_camera = pFrame_reference_->camera_->pixel2camera(Sophus::Vector2d(keypoints_curr_frame_[i].pt.x,keypoints_curr_frame_[i].pt.y), depth);
        point_ref_frame_.push_back(cv::Point3f(point_camera(0,0),
                                               point_camera(1,0),
                                               point_camera(2,0)));
        descriptors_ref_.push_back(descriptors_curr_.row(i));
      }
    }
    return 0;
  }
  
  int VisualOdometry::poseEstimationPnP()
  {
    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point2f> points_2d;
    for(cv::DMatch m:feature_matches_)
    {
      points_3d.push_back(point_ref_frame_[m.queryIdx]);
      points_2d.push_back(keypoints_curr_frame_[m.trainIdx].pt);
    }

    float fx, fy, cx, cy;
    fx = pFrame_reference_->camera_->get_fx();
    fy = pFrame_reference_->camera_->get_fy();
    cx = pFrame_reference_->camera_->get_cx();
    cy = pFrame_reference_->camera_->get_cy();
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                       fx, 0, cx,
                       0, fy, cy,
                       0,  0,  1);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(points_3d, points_2d, K,
                       cv::Mat(), rvec, tvec,
                       false, 100, 4.0,
                       0.99, inliers);
    num_inliers_ = inliers.rows;
    std::cout << "PnP inliers: " << num_inliers_ << std::endl;
    std::cout << "rvec: " << rvec << std::endl;
    std::cout << "tvec: " << tvec << std::endl;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    std::cout << R << std::endl;
    Eigen::Matrix3d r;
    Eigen::Vector3d t;
    cv::cv2eigen(R, r);
    cv::cv2eigen(tvec, t);

    Eigen::AngleAxisd angle(r);
    Eigen::Quaterniond q(r);
    //std::cout << "[Bebug] Before set_tcw_estimate" << std::endl;
    //T_curr_ref_estimated_ = Sophus::SE3<double>(Sophus::SO3<double>(q), Eigen::Vector3d(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)));
    T_curr_ref_estimated_ = Sophus::SE3<double>(r, t);
    //std::cout << "[Bebug] After set_tcw_estimate" << std::endl;
    //std::cout << "Tcr: " << tcre << std::endl; 
    //setTransformationEstimation(tcre);
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
    map_->insertKeyFrame(pFrame_current_);
    return 0;
  }
} // END of namespace myslam
