#ifndef __MAPPOINT_HH__
#define __MAPPOINT_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: mappoint.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:47:05
  * @last_modified_date: 2018-08-24 08:21:02
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>

// Declaration
namespace myslam
{
  /**
   * @ingroup DataType
   * @brief A class for beacon points in the world
   */
  class MapPoint
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      typedef std::shared_ptr<MapPoint> Ptr;    /*!< A shared_pointer points MapPoint*/
      MapPoint() = default;

      /**
       * @brief A constructor for MapPoint with point info
       * @param[in] id An identification of this map point
       * @param[in] position The coordinate of this map point
       * @param[in] norm The normalized direction vecter of this map point
       */
      MapPoint(long id,
               Eigen::Vector3d position,
               Eigen::Vector3d norm);
      virtual ~MapPoint() = default;

      /**
       * @brief Create a new MapPoint(on heap memory), and return a shared_pointer
       * @return A shared_pointer points this new instance of MapPoint
       */
      static MapPoint::Ptr createMapPoint();

      // Get methods

      /**
       * @brief Get the identification of this MapPoint
       */
      inline unsigned long get_id() const
      { return this->id_; }

      /**
       * @brief Get the position of this MapPoint
       */
      inline const Eigen::Vector3d& get_position() const
      { return this->position_; }

      /**
       * @brief Get the direction of this MapPoint
       */
      inline const Eigen::Vector3d& get_norm_direction() const
      { return this->norm_direction_; }

      /**
       * @brief Get the descriptor of this MapPoint
       */
      inline const cv::Mat& get_descriptor_() const
      { return this->descriptor_; }

      /**
       * @brief Get the observed times of this MapPoint
       */
      inline int get_observed_times() const
      { return this->observed_times_; }

      /**
       * @brief Get the correct times of this MapPoint
       */
      inline int get_correct_times() const
      { return this->correct_times_; }

    private:
      unsigned long id_;                /*!< Identification of this MapPoint*/
      Eigen::Vector3d position_;        /*!< Position of this MapPoint */
      Eigen::Vector3d norm_direction_;  /*!< A unit vector of direction of this MapPoint */
      cv::Mat descriptor_;              /*!< The descriptor for this MapPoint */
      int observed_times_;              /*!< A counter for observed times of this MapPoint */
      int correct_times_;               /*!< A counter for correct times of this MapPoint */
  };
}   // END of namespace myslam
#endif // __MAPPOINT_HH__
