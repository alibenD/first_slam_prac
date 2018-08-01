#ifndef __CONFIG_HH__
#define __CONFIG_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: config.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:18:58
  * @last_modified_date: 2018-08-01 10:28:39
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
   * @brief A class for Config
   */
  class Config
  {
    public:
      Config() = default;
      virtual ~Config() = default;
      static int setParameterFile(const std::string& filename);
      template<typename T>
        static T get(const std::string& key)
        {
          return T( Config::config_->file_[key] );
        }

    private:
      static std::shared_ptr<Config> config_;
      cv::FileStorage file_;
  };
}   // END of namespace myslam
#endif // __CONFIG_HH__
