/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 19:38:13
  * @last_modified_date: 2018-09-16 22:12:44
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/frame.hh>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//CODE
int main(int argc, char** argv)
{
  std::cout << argv[0] << std::endl;
  if(argc != 2)
  {
    std::cout << "Usage: ./test_frame dataset_dir" << std::endl;
    return -1;
  }
  cv::namedWindow("Show", cv::WINDOW_NORMAL);

  std::string path(argv[1]);
  std::cout << path << std::endl;
  std::ifstream fin(path + "/associate.txt");
  std::vector<myslam::Frame::Ptr> frame_ptrs;
  cv::Ptr<cv::ORB> p_orb = cv::ORB::create(400);

  while(!fin.eof())
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
    std::string image_filename = path + "/" + rgb_file;
    cv::Mat image = cv::imread(image_filename);
    auto pFrame = myslam::Frame::createFrame(image);
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    p_orb->detect(pFrame->getImage(), keypoints);
    p_orb->compute(pFrame->getImage(), keypoints, descriptors);
    pFrame->setKeypoints(keypoints);
    pFrame->setDescriptors(descriptors);
    cv::drawKeypoints(image,
                      keypoints,
                      image,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    frame_ptrs.push_back(pFrame);
    cv::imshow("Show", image);
    cv::waitKey(30);
  }

  //auto pFrame = myslam::Frame::createFrame(video_frame);

  std::cout << "Frame number: " << myslam::Frame::number_ << std::endl;
  return 0;
}
