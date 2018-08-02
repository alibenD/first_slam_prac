#ifndef __CAMERA_HH__
#define __CAMERA_HH__
/** Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: camera.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-07-31 20:49:07
  * @last_modified_date: 2018-08-01 17:10:56
  * @brief: Declaration of Camera class
  */

// Header include
#include <myslam/common.hh>

// Declaration

/**
 * @brief This is a namespace myslam
 * @details HAHAHAHAHAH
 */
namespace myslam
{
/**
 * @defgroup Sensor
 * @brief Sensor module including all sensors slam will use
 */   // END of group Sensor

/**
 * @ingroup Sensor
 * @brief A class for camera entities
 */
class Camera
{
  public:
    Camera() = default;
    Camera(float fx, float fy,
           float cx, float cy, float depth_scale=0)
      : fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy)
    {};
    virtual ~Camera() = default;

    // TF functions: world, camera, pixel

    /**
     * @brief A transformation of coordinates from world to camera.
     * @param[IN] point_world The coordinate of this point in the world frame.
     * @param[IN] T_camera_world The transformation(Lie Algebra) from worl to camera.
     * @retval coor_point_camera The coordinate of this point in camera
     */
    Eigen::Vector3d world2camera(const Eigen::Vector3d& point_world,
                                 const Sophus::SE3<double>& T_camera_world);

    /**
     * @brief A transformation of coordinate from world to pixel
     * @param[IN] point_world The coordinate of this point in the world frame.
     * @param[IN] T_camera_world The transformation(Lie Algebra) from camera to world.
     * @retval coor_point_pixel The coordinate of this point in pixel
     */
    Eigen::Vector2d world2pixel(const Eigen::Vector3d& point_world,
                                const Sophus::SE3<double>& T_camera_world);

    /**
     * @brief A transformation of coordinate from camera to world
     * @param[IN] point_camera The coordinate of this point in the world frame.
     * @param[IN] T_world_camera The transformation(Lie Algebra) from camera to world.
     * @retval coor_point_world The coordinate of this point in world
     */
    Eigen::Vector3d camera2world(const Eigen::Vector3d& point_camera,
                                 const Sophus::SE3<double>& T_world_camera);

    /**
     * @brief A transformation of coordinate from camera to pixel
     * @param[IN] point_camera The coordinate of this point in the world frame.
     * @retval coor_point_pixel The coordinate of this point in pixel
     */
    Eigen::Vector2d camera2pixel(const Eigen::Vector3d& point_camera);


    /**
     * @brief A transformation of coordinate from pixel to camera
     * @param[IN] point_pixel The coordinate of this point in the pixel frame.
     * @param[IN] depth a
     * @retval coor_point_camera The coordinate of this point in camera
     */
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d& point_pixel,
                                 double depth=1);

    /**
     * @brief A transformation of coordinate from pixel to world
     * @param[IN] point_pixel The coordinate of this point in the pixel frame.
     * @param[IN] T_camera_world The transformation(Lie Algebra) from worl to camera.
     * @param[IN] depth a
     * @retval coor_point_world The coordinate of this point in world
     */
    Eigen::Vector3d pixel2world(const Eigen::Vector2d& point_pixel,
                                const Sophus::SE3<double>& T_camera_world,
                                double depth=1);

    inline float get_depth_scale() const
    { return depth_scale_; }


  public:
    typedef std::shared_ptr<Camera> Ptr;    /*!< A pointer points this instance*/

  private:
    float fx_;  /*!< Focal distance by alpha(x axis direction)*/
    float fy_;  /*!< Focal distance by beta(y axis direction)*/
    float cx_;  /*!< Coordinates of optic center(x)*/
    float cy_;  /*!< Coordinates of optic center(y)*/
    float depth_scale_; /*!< Scale of depth */
};

}   // END of namespace myslam
#endif // __CAMERA_HH__