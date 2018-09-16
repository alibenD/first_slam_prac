/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-10 15:32:58
  * @last_modified_date: 2018-09-10 21:31:47
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
#include <opencv2/features2d/features2d.hpp>

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
  std::vector<myslam::Frame::Ptr> p_frames;

  cv::Ptr<cv::ORB> p_orb = cv::ORB::create(400);

  std::vector<cv::KeyPoint> keypoint_last, keypoint_current;
  cv::Mat descriptor_last, descriptor_current;
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


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
    std::cout << *p_frame << std::endl;
    p_orb->detect(p_frame->get_color(), keypoint_current);
    p_orb->compute(p_frame->get_color(), keypoint_current, descriptor_current);

    p_frame->set_descriptor(descriptor_current);
    p_frame->set_keypoint(keypoint_current);

    p_frames.push_back(p_frame);
    int size = p_frames.size();
    if(size < 2)
    {
      continue;
    }

    auto descriptor_last = p_frames[size-2]->get_descriptor();
    auto keypoint_last = p_frames[size-2]->get_keypoint();
    auto image_last = p_frames[size-2]->get_color();

    std::vector<cv::DMatch> matches;
    matcher->match(descriptor_last, descriptor_current, matches);

    // GOOD matches
    double min_dist = 10000, max_dist = 0.;
    std::vector<cv::DMatch> good_matches;
    for(int i=0; i < matches.size(); i++)
    {
      double dist = matches[i].distance;
      if(dist < min_dist)
      {
        min_dist = dist;
      }
      if(dist > max_dist)
      {
        max_dist = dist;
      }
    }

    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_dist*1.5, 30.0))
      {
        good_matches.push_back(m);
      }
    }

    cv::Mat matched_image;
    cv::drawMatches(image_last, keypoint_last, image, keypoint_current, good_matches, matched_image);
    cv::imshow("Show Image", matched_image);
    //cv::imshow("Show Image", p_frame->get_color());
    cv::waitKey(30);

  }
  return 0;
}
