/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_estimate.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-10 15:32:58
  * @last_modified_date: 2018-09-14 09:28:14
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <iostream>
#include <myslam.hh>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sophus/se3.hpp>
#include <myslam/pose_estimate_3d2d.hh>
#include <myslam/pose_estimate_2d2d.hh>

//CODE
int main(int argc, char** argv)
{
  if(argc != 3)
  {
    std::cout << "Usage: ./test_estimate ../config/YOUR.yml 30" << std::endl;
    return -1;
  }

  //myslam::Camera camera(argv[1]);
  myslam::Camera::Ptr p_camera = std::make_shared<myslam::Camera>(argv[1]);
  std::string dataset_dir = p_camera->get<std::string>("dataset_dir", *p_camera);
  std::cout << "Dataset: " << dataset_dir << std::endl;
  std::ifstream fin(dataset_dir + "/associate.txt");
  std::vector<myslam::Frame::Ptr> p_frames;

  cv::Ptr<cv::ORB> p_orb = cv::ORB::create(400);

  std::vector<cv::KeyPoint> keypoint_last, keypoint_current;
  cv::Mat descriptor_last, descriptor_current;
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


  cv::namedWindow("Show Image", cv::WINDOW_NORMAL);
  cv::viz::Viz3d vis("Visual Odometry");
  cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(2.0);
  vis.showWidget("World", world_coor);
  vis.showWidget("Camera", camera_coor);

  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);

  cv::Point3d camera_pos(10.0, 10.0, 10.0), camera_focal_point(0, 0, 0);
  cv::Point3d camera_y_dir(-1,0,0);
  cv::Affine3d camera_pose = cv::viz::makeCameraPose(camera_pos, camera_focal_point, camera_y_dir);
  vis.setViewerPose(camera_pose);

  while( !fin.eof() )
  {
    if(fin.good() == false)
    {
      break;
    }
    std::string rgb_time, rgb_file, depth_time, depth_file;
    fin >> rgb_time
        >> rgb_file
        >> depth_time
        >> depth_file;
    if(fin.eof())
    {
      break;
    }
    //std::cout << "RGB time: " << rgb_time << std::endl
    //          << "RGB file: " << rgb_file << std::endl
    //          << "Depth time: " << depth_time << std::endl
    //          << "Depth file: " << depth_file << std::endl;
    std::string image_filename = dataset_dir + "/" + rgb_file;
    std::string depth_filename = dataset_dir + "/" + depth_file;

    double timestamp = atof(rgb_time.c_str());
    cv::Mat image = cv::imread(image_filename);
    cv::Mat depth = cv::imread(depth_filename);
    std::vector<cv::Point3f> point3d;
    std::vector<cv::Point2f> point2d;

    auto p_frame = myslam::Frame::createFrame(image, p_camera, timestamp);
    p_frame->set_depth(depth);
    //std::cout << *p_frame << std::endl;
    p_orb->detect(p_frame->get_color(), keypoint_current);
    p_orb->compute(p_frame->get_color(), keypoint_current, descriptor_current);

    for(auto& keypoint : keypoint_current)
    {
      int x = cvRound(keypoint.pt.x);
      int y = cvRound(keypoint.pt.y);

      auto d = depth.ptr<unsigned short>(y)[x];
      int dx[4] = {-1, 0, 1, 0};
      int dy[4] = {0, -1, 0, 1};

      if(d == 0)
      {
        continue;
        for(int i = 0; i<4; i++)
        {
          d = depth.ptr<unsigned short>(y+dy[i])[x+dx[i]];
          if(d == 0)
          {
            d = -1;
          }
        }
      }
      float dd = d/p_camera->get_depth_scale();
      double xx = (x - p_camera->get_cx()) * dd / p_camera->get_fx();
      double yy = (x - p_camera->get_cy()) * dd / p_camera->get_fy();

      cv::Point2f t_point2d(x,y);
      point2d.push_back(t_point2d);

      cv::Point3f t_point3d(xx,yy,dd);
      point3d.push_back(t_point3d);
    }

    p_frame->set_descriptor(descriptor_current);
    p_frame->set_keypoint(keypoint_current);
    p_frame->set_point3d(point3d);
    p_frames.push_back(p_frame);
    int size = p_frames.size();
    if(size < 2)
    {
      //std::cout << *p_frame << std::endl;
      continue;
    }

    auto descriptor_last = p_frames[size-2]->get_descriptor();
    auto keypoint_last = p_frames[size-2]->get_keypoint();
    auto image_last = p_frames[size-2]->get_color();

    std::vector<cv::DMatch> matches;
    matcher->match(descriptor_last, descriptor_current, matches);

    //std::cout << "SizeMatch: " << matches.size();
    //std::cout << "Size3d: " << point3d.size()
    //          << "\tSize2d: " << point2d.size();

    p_frame->set_point2d(point2d);
    p_frame->set_point3d(point3d);

    // GOOD matches
    double min_dist = 10000, max_dist = 0.;
    std::vector<cv::DMatch> good_matches;
    for(int i=0; i < matches.size(); i++)
    {
      double dist = matches[i].distance;
      if(dist < min_dist)
      {
        min_dist = dist;
      }
      if(dist > max_dist)
      {
        max_dist = dist;
      }
    }

    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_dist*1.5, 30.0))
      {
        good_matches.push_back(m);
      }
    }

    std::vector<cv::Point3f> point3d_good;
    std::vector<cv::Point2f> point2d_good;
    auto point3d_last = p_frames[size-2]->get_point3d();
    double max = 0;
    for(auto match : good_matches)
    {
      auto x = int(keypoint_current[match.queryIdx].pt.x);
      auto y = int(keypoint_current[match.queryIdx].pt.y);

      auto d = depth.ptr<unsigned short>(y)[x];
      int dx[4] = {-1, 0, 1, 0};
      int dy[4] = {0, -1, 0, 1};

      if(d == 0)
      {
        continue;
      }
      float dd = d/p_camera->get_depth_scale();
      //std::cout << dd << std::endl;
      auto xx = (keypoint_current[match.queryIdx].pt.x - p_camera->get_cx()) * dd / p_camera->get_fx();
      auto yy = (keypoint_current[match.queryIdx].pt.y - p_camera->get_cy()) * dd / p_camera->get_fy();

      cv::Point2f t_point2d(x,y);
      point2d_good.push_back(t_point2d);

      cv::Point3f t_point3d(xx,yy,dd);
      point3d_good.push_back(point3d_last[match.queryIdx]);
    }

    cv::Mat rotation_vector, translation, inliers;
    cv::solvePnPRansac(point3d_good,
                       point2d_good,
                       p_camera->get_intrinsic(),
                       cv::Mat(),
                       rotation_vector,
                       translation,
                       false);
    //std::cout << "PnP inlier: " << inliers.rows << std::endl;

    cv::Mat rotation;
    cv::Rodrigues(rotation_vector, rotation);
    std::cout << rotation_vector << std::endl;
    //pose_estimate(keypoint_last, keypoint_current, good_matches, p_camera->get_intrinsic(), rotation, translation);
    //std::cout << "Rotation:\n" << rotation << std::endl;
    //std::cout << "Translation:\n" << translation << std::endl;

    std::cout << "point3d_good: " << point3d_good.size()
              << "\tpoint2d_good: " << point2d_good.size() << std::endl;

    // Optimizer
    //bundleAdjustment(point3d_good,
    //                 point2d_good,
    //                 p_camera->get_intrinsic(),
    //                 inliers,
    //                 rotation,
    //                 translation);
    bundleAdjustment(point3d_good,
                     point2d_good,
                     p_camera->get_intrinsic(),
                     rotation,
                     translation);

    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_vector;

    cv::cv2eigen(rotation, rotation_matrix);
    cv::cv2eigen(translation, translation_vector);

    Sophus::SE3<double> transform_delta(rotation_matrix, translation_vector);
    auto transform = transform_delta * p_frames[size-2]->get_Tcw();
    auto transform_inv = transform.inverse();
    p_frame->set_Tcw(transform);
    //std::cout << "Matrix:\n" << p_frame->get_Tcw().matrix() << std::endl;
    //std::cout << "Inv_Matrix:\n" << transform_inv.matrix() << std::endl;
    //std::cout << *p_frame << std::endl;

    cv::Mat matched_image;
    cv::drawMatches(image_last, keypoint_last, image, keypoint_current, good_matches, matched_image);
    cv::imshow("Show Image", matched_image);
    //cv::imshow("Show Image", p_frame->get_color());

    cv::Affine3d M(cv::Affine3d::Mat3(
                      transform_inv.rotationMatrix()(0,0), transform_inv.rotationMatrix()(0,1), transform_inv.rotationMatrix()(0,2),
                      transform_inv.rotationMatrix()(1,0), transform_inv.rotationMatrix()(1,1), transform_inv.rotationMatrix()(1,2),
                      transform_inv.rotationMatrix()(2,0), transform_inv.rotationMatrix()(2,1), transform_inv.rotationMatrix()(2,2)
                      ),
                   cv::Affine3d::Vec3(transform_inv.translation()(0,0), transform_inv.translation()(1,0), transform_inv.translation()(2,0)));
                   //cv::Affine3d::Vec3(1,1,1));
    vis.setWidgetPose("Camera", M);

    cv::waitKey(atoi(argv[2]));
    //cv::waitKey(30);
    vis.spinOnce(1, false);
  }
  return 0;
}
