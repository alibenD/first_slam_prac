#ifndef __CAMERA_HH__
#define __CAMERA_HH__
/** Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: camera.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-07-31 20:49:07
  * @last_modified_date: 2018-09-14 09:30:06
  * @brief: Declaration of Camera class
  */

// Header include
#include <myslam/common.hh>
#include <string>

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

    Camera(const std::string& config_path);
    /**
     * @brief A constructor with camera parameter
     * @param[in] fx Focal distance by Alpha(x axis)
     * @param[in] fy Focal distance by Beta(y axis)
     * @param[in] cx Coordinates of optic center at x axis
     * @param[in] cy Coordinates of optic center at y axis
     * @param[in] depth_scale Scale of depth
     */
    Camera(float fx, float fy,
           float cx, float cy, float depth_scale=0)
      : fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy),
        depth_scale_(depth_scale)
    {};
    virtual ~Camera() = default;
    friend std::ostream& operator<<(std::ostream& os, const Camera& camera);

    // TF functions: world, camera, pixel

    /**
     * @brief A transformation of coordinates from world to camera.
     * @param[in] point_world The coordinate of this point in the world frame.
     * @param[in] T_camera_world The transformation(Lie Algebra) from worl to camera.
     * @return coor_point_camera The coordinate of this point in camera
     */
    Eigen::Vector3d world2camera(const Eigen::Vector3d& point_world,
                                 const Sophus::SE3<double>& T_camera_world);

    /**
     * @brief A transformation of coordinate from world to pixel
     * @param[in] point_world The coordinate of this point in the world frame.
     * @param[in] T_camera_world The transformation(Lie Algebra) from camera to world.
     * @return coor_point_pixel The coordinate of this point in pixel
     */
    Eigen::Vector2d world2pixel(const Eigen::Vector3d& point_world,
                                const Sophus::SE3<double>& T_camera_world);

    /**
     * @brief A transformation of coordinate from camera to world
     * @param[in] point_camera The coordinate of this point in the world frame.
     * @param[in] T_world_camera The transformation(Lie Algebra) from camera to world.
     * @return coor_point_world The coordinate of this point in world
     */
    Eigen::Vector3d camera2world(const Eigen::Vector3d& point_camera,
                                 const Sophus::SE3<double>& T_world_camera);

    /**
     * @brief A transformation of coordinate from camera to pixel
     * @param[in] point_camera The coordinate of this point in the world frame.
     * @return coor_point_pixel The coordinate of this point in pixel
     */
    Eigen::Vector2d camera2pixel(const Eigen::Vector3d& point_camera);


    /**
     * @brief A transformation of coordinate from pixel to camera
     * @param[in] point_pixel The coordinate of this point in the pixel frame.
     * @param[in] depth a
     * @return coor_point_camera The coordinate of this point in camera
     */
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d& point_pixel,
                                 double depth=1);

    /**
     * @brief A transformation of coordinate from pixel to world
     * @param[in] point_pixel The coordinate of this point in the pixel frame.
     * @param[in] T_camera_world The transformation(Lie Algebra) from worl to camera.
     * @param[in] depth 
     * @return coor_point_world The coordinate of this point in world
     */
    Eigen::Vector3d pixel2world(const Eigen::Vector2d& point_pixel,
                                const Sophus::SE3<double>& T_camera_world,
                                double depth=1);

    // Get methods

    /**
     * @brief Get Focal distance(x axis) of this camera
     * @return The value of focal distance(x axis)
     */
    inline float get_fx() const
    { return fx_; }

    /**
     * @brief Get Focal distance(y axis) of this camera
     * @return The value of focal distance(y axis)
     */
    inline float get_fy() const
    { return fy_; }

    /**
     *  @brief Get the coordinate of optic center on x axis
     *  @return The value of coor x
     */
    inline float get_cx() const
    { return cx_; }

    /**
     *  @brief Get the coordinate of optic center on y axis
     *  @return The value of coor y
     */
    inline float get_cy() const
    { return cy_; }

    /**
     * @brief Get scale of depth of this camera
     * @return A value of scale of depth
     */
    inline float get_depth_scale() const
    { return depth_scale_; }

    /**
     * @brief Get the matrix of intrinsic
     */
    inline cv::Mat get_intrinsic() const
    { return intrinsic_; }

    /**
     * @brief A template static function for getting different type of setting items
     * @param[in] key The key of an item
     * @return The value of this item
     */
    template<typename T>
      static T get(const std::string& key, const Camera& camera)
      {
        return T( camera.file_[key] );
      }

  public:
    typedef std::shared_ptr<Camera> Ptr;    /*!< A shared_pointer points Camera*/

  protected:

  private:
    float fx_;  /*!< Focal distance by alpha(x axis direction)*/
    float fy_;  /*!< Focal distance by beta(y axis direction)*/
    float cx_;  /*!< Coordinates of optic center(x)*/
    float cy_;  /*!< Coordinates of optic center(y)*/
    float depth_scale_; /*!< Scale of depth */
    cv::Mat intrinsic_;
    cv::FileStorage file_;
};

}   // END of namespace myslam
#endif // __CAMERA_HH__