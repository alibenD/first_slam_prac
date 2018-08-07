/** Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: camera.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-07-31 20:49:07
  * @last_modified_date: 2018-08-07 16:24:49
  * @brief: Definition of Camera class
  */

//INCLUDE
#include <myslam/camera.hh>
#include <myslam/config.hh>

//CODE
namespace myslam
{
  Camera::Camera()
  {
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
  }

  Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d& point_world, const Sophus::SE3<double>& T_camera_world)
  {
    Eigen::Vector3d coor_point_camera = T_camera_world*point_world;
    return coor_point_camera;
  }

  Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d& point_world, const Sophus::SE3<double>& T_camera_world)
  {
    Eigen::Vector2d coor_point_pixel = camera2pixel(T_camera_world*point_world);
    return coor_point_pixel;
  }

  Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d& point_camera, const Sophus::SE3<double>& T_world_camera)
  {
    Eigen::Vector3d coor_point_world = T_world_camera.inverse()*point_camera;
    return coor_point_world;
  }

  Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d& point_camera)
  {
    float x_tmp = fx_*point_camera(0,0) / point_camera(2,0) + cx_;
    float y_tmp = fy_*point_camera(1,0) / point_camera(2,0) + cy_;
    Eigen::Vector2d coor_point_pixel(x_tmp, y_tmp);
    return coor_point_pixel;
  }

  Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d& point_pixel, double depth)
  {
    float x_tmp = (point_pixel(0,0) - cx_) * depth / fx_;
    float y_tmp = (point_pixel(1,0) - cy_) * depth / fy_;
    Eigen::Vector3d coor_point_camera(x_tmp, y_tmp, depth);
    return coor_point_camera;
  }

  Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d& point_pixel, const Sophus::SE3<double>& T_camera_world, double depth)
  {
    Eigen::Vector3d coor_point_world = camera2world( pixel2camera(point_pixel, depth), T_camera_world );
    return coor_point_world;
  }
}   // END of namespace myslam
