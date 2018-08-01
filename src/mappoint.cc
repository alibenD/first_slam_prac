/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: mappoint.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 13:25:24
  * @last_modified_date: 2018-08-01 13:31:55
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/mappoint.hh>

//CODE
namespace myslam
{
  MapPoint::MapPoint(long id,
                     Eigen::Vector3d position,
                     Eigen::Vector3d norm_direction)
    : id_(id),
      position_(position),
      norm_direction_(norm_direction),
      observed_times_(0),
      correct_times_(0)
  {
  }

  MapPoint::Ptr MapPoint::createMapPoint()
  {
    static long factory_id = 0;
    return MapPoint::Ptr(new MapPoint(factory_id++,
                                      Eigen::Vector3d(0,0,0),
                                      Eigen::Vector3d(0,0,0)));
  }
}   // END of namespace myslam
