/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:30:37
  * @last_modified_date: 2018-08-16 12:59:16
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/frame.hh>

//CODE
namespace myslam
{
  Frame::Frame(long id,
               double timestamp,
               Sophus::SE3<double> T_camera_world,
               Camera::Ptr camera,
               cv::Mat color,
               cv::Mat depth)
    : id_(id),
      timestamp_(timestamp),
      T_camera_world_(T_camera_world),
      camera_(camera)
      //color_(color.clone()),
      //depth_(depth.clone())
  {
    color.copyTo(color_);
    depth.copyTo(depth_);
  }

  Frame::Ptr Frame::createFrame()
  {
    static unsigned long factory_id = 0;
    //return Frame::Ptr(new Frame(factory_id++));
    //return std::make_shared<Frame>(factory_id++);
    auto pFrame = std::make_shared<Frame>(factory_id);
    std::cout << "Frame ID: " << factory_id << std::endl;
    factory_id++;
    return pFrame;
  }

  double Frame::findDepth(const cv::KeyPoint& key_point)
  {
    int x = cvRound(key_point.pt.x);
    int y = cvRound(key_point.pt.y);
    unsigned short depth = depth_.ptr<unsigned short>(y)[x];
    if(depth != 0)
    {
      return double(depth)/camera_->get_depth_scale();
    }
    else
    {
      int dx[4] = {-1, 0, 1, 0};
      int dy[4] = {0, -1, 0, 1};
      for(int i=0; i<4; i++)
      {
        depth = depth_.ptr<unsigned short>(y+dy[i])[x+dx[i]];
        if(depth != 0)
        {
          return double(depth)/camera_->get_depth_scale();
        }
      }
    }
    return -1.0;
  }

  Eigen::Vector3d Frame::getCameraCenter() const
  {
    return T_camera_world_.inverse().translation();
  }

  bool Frame::isInFrame(const Eigen::Vector3d& point_world)
  {
    Eigen::Vector3d point_camera = camera_->world2camera(point_world, T_camera_world_);
    if(point_world(2, 0) < 0)
    {
      return false;
    }
    Eigen::Vector2d point_pixel = camera_->world2pixel(point_world, T_camera_world_);
    return point_pixel(0,0)>0 &&
           point_pixel(1,0)>0 &&
           point_pixel(0,0)<color_.cols &&
           point_pixel(1,0)<color_.rows;
  }

}   // END of namespace myslam
