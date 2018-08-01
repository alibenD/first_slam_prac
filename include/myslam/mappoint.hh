#ifndef __MAPPOINT_HH__
#define __MAPPOINT_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: mappoint.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 09:47:05
  * @last_modified_date: 2018-08-01 10:12:15
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
      MapPoint() = default;
      MapPoint(long id,
               Eigen::Vector3d position,
               Eigen::Vector3d norm);
      virtual ~MapPoint() = default;
      static MapPoint::Ptr createMapPoint();

    public:
      typedef std::shared_ptr<MapPoint> Ptr;

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
