#ifndef __MAPPOINT_HH__
#define __MAPPOINT_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: mappoint.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:47:05
  * @last_modified_date: 2018-08-01 17:07:13
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
      typedef std::shared_ptr<MapPoint> Ptr;
      MapPoint() = default;
      MapPoint(long id,
               Eigen::Vector3d position,
               Eigen::Vector3d norm);
      virtual ~MapPoint() = default;
      static MapPoint::Ptr createMapPoint();

      // Get methods
      inline unsigned long get_id() const
      { return this->id_; }

      inline const Eigen::Vector3d& get_position() const
      { return this->position_; }

      inline const Eigen::Vector3d& get_norm_direction() const
      { return this->norm_direction_; }

      inline const cv::Mat& get_descriptor_() const
      { return this->descriptor_; }

      inline int get_ovserved_times() const
      { return this->observed_times_; }

      inline int get_correct_times() const
      { return this->correct_times_; }

    private:
      unsigned long id_;
      Eigen::Vector3d position_;
      Eigen::Vector3d norm_direction_;
      cv::Mat descriptor_;
      int observed_times_;
      int correct_times_;
  };
}   // END of namespace myslam
#endif // __MAPPOINT_HH__
