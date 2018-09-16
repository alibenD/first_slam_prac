/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_camera.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-10 14:18:54
  * @last_modified_date: 2018-09-10 14:54:32
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam.hh>
#include <iostream>

//CODE
int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cout << "Usage: ./test_camera ../config/YOUR.yml" << std::endl;
    return -1;
  }

  myslam::Camera camera(argv[1]);
  std::cout << camera << std::endl;
  return 0;
}
