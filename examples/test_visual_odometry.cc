/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-16 23:20:27
  * @last_modified_date: 2018-09-16 23:38:06
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/visual_odometry.hh>

//CODE
int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cout << "Usage: ./test_frame dataset_dir" << std::endl;
    return -1;
  }
  cv::namedWindow("Show", cv::WINDOW_NORMAL);
  myslam::VisualOdometry vo(400);
  
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
    vo.addFrame(pFrame);
  }
  auto frames = vo.getFrames();
  std::cout << "Frames: " << frames.size() << std::endl;
  for(auto frame : frames)
  {
    auto image = frame->getImage().clone();
    auto keypoints = frame->getKeypoints();
    cv::drawKeypoints(image,
                      keypoints,
                      image,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    cv::imshow("Show", image);
    cv::waitKey(30);
  }
  return 0;
}
