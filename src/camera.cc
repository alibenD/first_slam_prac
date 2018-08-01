/** Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: camera.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-07-31 20:49:07
  * @last_modified_date: 2018-08-01 09:01:41
  * @brief: Definition of Camera class
  */

//INCLUDE
#include <myslam/camera.hh>

//CODE
namespace myslam
{
    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d& point_world, const Sophus::SE3& T_camera_world)
    {
      Eigen::Vector3d coor_point_camera;
      return coor_point_camera;
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d& point_world, const Sophus::SE3& T_camera_world)
    {
      Eigen::Vector2d coor_point_pixel;
      return coor_point_pixel;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d& point_camera, const Sophus::SE3& T_world_camera)
    {
      Eigen::Vector3d coor_point_world;
      return coor_point_world;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d& point_camera)
    {
      Eigen::Vector2d coor_point_pixel;
      return coor_point_pixel;
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d& point_pixel, double depth=1)
    {
      Eigen::Vector3d coor_point_camera;
      return coor_point_camera;
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d& point_pixel, const Sophus::SE3& T_camera_world, double depth=1)
    {
      Eigen::Vector3d coor_point_world;
      return coor_point_world;
    }
}   // END of namespace myslam
