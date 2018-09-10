/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-10 15:32:58
  * @last_modified_date: 2018-09-10 17:04:38
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <iostream>
#include <myslam.hh>
#include <fstream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//CODE
int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cout << "Usage: ./test_frame ../config/YOUR.yml" << std::endl;
    return -1;
  }

  //myslam::Camera camera(argv[1]);
  myslam::Camera::Ptr p_camera = std::make_shared<myslam::Camera>(argv[1]);
  std::string dataset_dir = p_camera->get<std::string>("dataset_dir", *p_camera);
  std::cout << "Dataset: " << dataset_dir << std::endl;
  std::ifstream fin(dataset_dir + "/associate.txt");

  cv::namedWindow("Show Image", cv::WINDOW_NORMAL);

  while( !fin.eof() )
  {
    if(fin.good() == false)
    {
      break;
    }
    std::string rgb_time, rgb_file, depth_time, depth_file;
    fin >> rgb_time
        >> rgb_file
        >> depth_time
        >> depth_file;
    if(fin.eof())
    {
      break;
    }
    //std::cout << "RGB time: " << rgb_time << std::endl
    //          << "RGB file: " << rgb_file << std::endl
    //          << "Depth time: " << depth_time << std::endl
    //          << "Depth file: " << depth_file << std::endl;
    std::string image_filename = dataset_dir + "/" + rgb_file;
    double timestamp = atof(rgb_time.c_str());
    cv::Mat image = cv::imread(image_filename);

    auto p_frame = myslam::Frame::createFrame(image, p_camera, timestamp);

    cv::imshow("Show Image", p_frame->get_color());
    cv::waitKey(30);

  }
  return 0;
}
