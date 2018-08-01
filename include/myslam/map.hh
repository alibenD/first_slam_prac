#ifndef __MAP_HH__
#define __MAP_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: map.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:08:31
  * @last_modified_date: 2018-08-01 10:19:43
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
   * @brief A class for map
   */
  class Map
  {
    public:
      Map() = default;
      virtual ~Map() = default;
      int insertKeyFrame(Frame::Ptr frame);
      int insertMapPoint(MapPoint::Ptr map_point);

    public:
      typedef std::shared_ptr<Map> Ptr;

    private:
      std::unordered_map<unsigned long, MapPoint::Ptr> map_points_;
      std::unordered_map<unsigned long, Frame::Ptr> key_frames_;
  };  // END of declaration of class Map
}   // END of namespace myslam
#endif // __MAP_HH__
