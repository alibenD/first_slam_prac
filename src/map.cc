/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: map.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:08:31
  * @last_modified_date: 2018-08-23 22:45:13
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/map.hh>

//CODE
namespace myslam
{
  int Map::insertKeyFrame(const Frame::Ptr& pFrame)
  {
    if(key_frames_.find(pFrame->get_id()) == key_frames_.end())
    {
      key_frames_.insert(std::make_pair(pFrame->get_id(), pFrame));
    }
    else
    {
      key_frames_[pFrame->get_id()] = pFrame;
    }
    std::cout << "Key frame size = " << key_frames_.size() << std::endl;
    return 0;
  }

  int Map::insertMapPoint(const MapPoint::Ptr& map_point)
  {
    if(map_points_.find(map_point->get_id()) == map_points_.end())
    {
      map_points_.insert(std::make_pair(map_point->get_id(), map_point));
    }
    else
    {
      map_points_[map_point->get_id()] = map_point;
    }
    return 0;
  }
}   // END of namespace myslam
