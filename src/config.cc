/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: config.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-01 13:32:48
  * @last_modified_date: 2018-08-01 17:15:25
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/config.hh>

//CODE
namespace myslam
{
  int Config::setParameterFile(const std::string& filename)
  {
    if(config_ == nullptr)
    {
      config_ = std::shared_ptr<Config>(new Config);
    }
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if(config_->file_.isOpened() == false)
    {
      std::cerr<< "parameter file " << filename
               << " does not exist." << std::endl;
      config_->file_.release();
      return 1;
    }
  }

  std::shared_ptr<Config> Config::config_ = nullptr;
}   // END of namespace myslam
