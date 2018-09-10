/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-12 00:28:49
  * @last_modified_date: 2018-08-23 22:32:11
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <myslam/frame.hh>

//CODE
int main(int argc, char** argv)
{
    //float RR[3][3] = {0.9999440783632033, 0.003838581309852994, -0.009854209247405444,
    //-0.003839529441099584, 0.9999926259920509, -7.729937386987676e-05,
    //0.009853839862456323, 0.0001151305776865186, 0.999951443113572};
    //  float rrv[3] = {-0.0007370704162898583, -0.01180572378082581, 0.00584823839696974};
    //  float rrt[3] = {0.01683331040938662, 0.004372147944822377, 0.004249924637175034};
    //cv::Mat R(3,3,CV_32FC1, RR);
    //cv::Mat rvec(3,1, CV_32FC1, rrv), tvec(3,1, CV_32FC1, rrt);
    //cv::Rodrigues(rvec, R);
    //Eigen::Matrix3d r;
    //Eigen::Vector3d t;
    //cv::cv2eigen(R, r);
    //cv::cv2eigen(tvec, t);

    //Eigen::AngleAxisd angle(r);
    //Eigen::Quaterniond q(r);
    //Sophus::SE3<double>(r, t);
    //std::cout << "Tcr: " << tcre << std::endl;
    //setTransformationEstimation(tcre);
  auto pFrame1 = myslam::Frame::createFrame();
  auto pFrame2 = myslam::Frame::createFrame();
  auto pFrame3 = myslam::Frame::createFrame();
  auto pFrame4 = myslam::Frame::createFrame();
  auto pFrame5 = myslam::Frame::createFrame();
  auto pFrame6 = myslam::Frame::createFrame();
  auto pFrame7 = myslam::Frame::createFrame();
  auto pFrame8 = myslam::Frame::createFrame();
  auto pFrame9 = myslam::Frame::createFrame();
  auto pFrame10 = myslam::Frame::createFrame();
  auto pFrame11 = myslam::Frame::createFrame();
  return 0;
}
