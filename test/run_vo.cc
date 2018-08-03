/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: run_vo.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-03 09:11:14
  * @last_modified_date: 2018-08-03 10:38:35
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam.hh>
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>

//CODE
int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: ./run_vo parameter_file" << std::endl;
    return 1;
  }
  myslam::Config::setParameterFile(argv[1]);
  myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

  std::string dataset_dir = myslam::Config::get<std::string>("dataset_dir");
  std::cout << "Dataset: " << dataset_dir << std::endl;
  std::ifstream fin(dataset_dir + "/associate.txt");

  if( !fin )
  {
    std::cout << "Please generate the associate file called associate.txt!" << std::endl;
    return 1;
  }

  std::vector<std::string> rgb_files, depth_files;
  std::vector<double> rgb_times, depth_times;
  while( !fin.eof() )
  {
    std::string rgb_time, rgb_file, depth_time, depth_file;
    fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
    rgb_times.push_back( atof(rgb_time.c_str()) );
    depth_times.push_back( atof(depth_time.c_str()) );
    rgb_files.push_back(dataset_dir + "/" + depth_file);

    if(fin.good() == false)
    {
      break;
    }
  }

  myslam::Camera::Ptr camera(new myslam::Camera);

  // Visualization
  cv::viz::Viz3d vis("Visual Odometry");
  cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
  cv::Point3d camera_pos(0, -1.0, -1.0), camera_focal_point(0, 0, 0);
  cv::Point3d camera_y_dir(0,1,0);
  cv::Affine3d camera_pose = cv::viz::makeCameraPose(camera_pos, camera_focal_point, camera_y_dir);
  vis.setViewerPose(camera_pose);

  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
  vis.showWidget("World", world_coor);
  vis.showWidget("Camera", camera_coor);

  std::cout << "Read total:" << rgb_files.size() << " entries" << std::endl;
  for(int i=0; i< rgb_files.size(); i++)
  {
    cv::Mat color = cv::imread(rgb_files[i]);
    cv::Mat depth = cv::imread(depth_files[i], cv::IMREAD_UNCHANGED);
    if( color.data==nullptr || depth.data==nullptr)
    {
      break;
    }
    myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->set_color(color);
    pFrame->set_depth(depth);
    pFrame->set_timestamp(rgb_times[i]);

    boost::timer timer;
    vo->addFrame(pFrame);
    std::cout << "VO costs time: " << timer.elapsed() << std::endl;

    if(vo->getStatus() == myslam::VisualOdometry::LOST)
    {
      break;
    }
    Sophus::SE3<double> T_w_c = pFrame->get_Tcw().inverse();

    cv::Affine3d M(cv::Affine3d::Mat3(
                      T_w_c.rotationMatrix()(0,0), T_w_c.rotationMatrix()(0,1), T_w_c.rotationMatrix()(0,2),
                      T_w_c.rotationMatrix()(0,3), T_w_c.rotationMatrix()(0,4), T_w_c.rotationMatrix()(0,5),
                      T_w_c.rotationMatrix()(0,6), T_w_c.rotationMatrix()(0,7), T_w_c.rotationMatrix()(0,8)
                      ),
                   cv::Affine3d::Vec3(T_w_c.translation()(0,0), T_w_c.translation()(1,0), T_w_c.translation()(2,0)));
    cv::imshow("Image", color);
    cv::waitKey(1);
    vis.setWidgetPose("Camera", M);
    vis.spinOnce(1, false);
  }
  return 0;
}
