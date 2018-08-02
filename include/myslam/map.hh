#ifndef __MAP_HH__
#define __MAP_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: map.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:08:31
  * @last_modified_date: 2018-08-02 09:51:35
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>
#include <myslam/map.hh>
#include <myslam/frame.hh>
#include <myslam/mappoint.hh>

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

      /**
       * @brief Insert a new key frame in this map
       * @param[in] frame A shared_pointer of a key frame
       */
      int insertKeyFrame(Frame::Ptr frame);

      /**
       * @brief Insert a new map point in this map
       * @param[in] frame A shared_pointer of a MapPoint
       */
      int insertMapPoint(MapPoint::Ptr map_point);

    public:
      typedef std::shared_ptr<Map> Ptr;   /*!< A datatype shared_pointer points Map*/

    private:
      std::unordered_map<unsigned long, MapPoint::Ptr> map_points_;   /*!< A set of map points*/
      std::unordered_map<unsigned long, Frame::Ptr> key_frames_;      /*!< A set of key frames*/
  };  // END of declaration of class Map
}   // END of namespace myslam
#endif // __MAP_HH__
