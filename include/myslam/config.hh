#ifndef __CONFIG_HH__
#define __CONFIG_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: config.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 10:18:58
  * @last_modified_date: 2018-08-02 10:19:14
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

      /**
       * @brief Given a path of config file, read settings from it.
       * @param[in] filename The path of a config file
       */
      static int setParameterFile(const std::string& filename);

      /**
       * @brief A template static function for getting different type of setting items
       * @param[in] key The key of an item
       * @return The value of this item
       */
      template<typename T>
        static T get(const std::string& key)
        {
          return T( Config::config_->file_[key] );
        }

    private:
      static std::shared_ptr<Config> config_;   /*!< A share_pointer points Config*/
      cv::FileStorage file_;                    /*!< A config file handle*/
  };
}   // END of namespace myslam
#endif // __CONFIG_HH__
