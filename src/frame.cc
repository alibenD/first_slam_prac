/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 12:28:57
  * @last_modified_date: 2018-09-16 20:23:31
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/frame.hh>

//CODE
namespace myslam
{
  long long Frame::number_ = 0;

  Frame::Frame(long long id)
    :Frame()
  {
    id_ = id;
  }

  Frame::Ptr Frame::createFrame(const cv::Mat& image)
  {
    Frame::Ptr p_frame = std::make_shared<Frame>(Frame::number_++);
    p_frame->setImage(image);
    return p_frame;
  }
}
