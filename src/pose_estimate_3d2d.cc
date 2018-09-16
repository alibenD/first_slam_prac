/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: pose_estimate_3d2d.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-09-11 22:52:55
  * @last_modified_date: 2018-09-14 09:15:39
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/pose_estimate_3d2d.hh>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <myslam/g2o_types.hh>
#include <chrono>

//CODE

void pose_estimate_3d2d(const std::vector<cv::KeyPoint>& keypoint_last,
                        const std::vector<cv::KeyPoint>& keypoint_current,
                        const std::vector<cv::Point3f>& spatial_point,
                        const std::vector<cv::DMatch>& matches,
                        const cv::Mat& intrinsic,
                        cv::Mat& rotation,
                        cv::Mat& t)
{
  std::vector<cv::Point2f> point2d;
}

void bundleAdjustment_depre(const std::vector<cv::Point3f>& point3d,
                      const std::vector<cv::Point2f>& point2d,
                      const cv::Mat& intrinsic,
                      const cv::Mat& inliers,
                      cv::Mat& R,
                      cv::Mat& t)
{
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
  g2o::SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<Block::PoseMatrixType>>(); // 线性方程求解器
  auto solver_ptr = g2o::make_unique<Block>(std::move(linearSolver));

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr ));

  optimizer.setAlgorithm ( solver );

  g2o::VertexSE3Expmap* pose= new g2o::VertexSE3Expmap();
  Eigen::Matrix3d rotation_matrix;
  Eigen::Vector3d translate_vector(t.at<double>(0,0),
                                   t.at<double>(1,0),
                                   t.at<double>(2,0));
  rotation_matrix << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(rotation_matrix, translate_vector));
  optimizer.addVertex(pose);

  //myslam::Camera* camera = new myslam::Camera(R.at<double>(0,0),
  //                                            R.at<double>(1,1),
  //                                            R.at<double>(0,2),
  //                                            R.at<double>(1,2),
  //                                            5000);

  int index = 1;
  for(auto& p : point3d)
  {
    g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
    point->setId(index++);
    point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
    point->setMarginalized(true);
    optimizer.addVertex(point);
  }

  g2o::CameraParameters* camera = new g2o::CameraParameters(517.0, Eigen::Vector2d(intrinsic.at<double>(0,2), intrinsic.at<double>(1,2)), 0);
  camera->setId(0);
  optimizer.addParameter(camera);

  index = 1;
  for(auto& p:point2d)
  {
    g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
    edge->setId(index);
    edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
    edge->setVertex(1, pose);
    edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
    edge->setParameterId(0, 0);
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    index++;
  }

  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  auto se3 = Sophus::SE3<double>(pose->estimate().rotation(), pose->estimate().translation());
  std::cout << "TF:\n" << se3.matrix();
  cv::eigen2cv(se3.rotationMatrix(), R);
  cv::eigen2cv(se3.translation(), t);
  delete solver;
}


void bundleAdjustment (
    const std::vector< cv::Point3f > points_3d,
    const std::vector< cv::Point2f > points_2d,
    const cv::Mat& K,
    cv::Mat& R, cv::Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    g2o::SparseOptimizer optimizer;
    //Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<Block::PoseMatrixType>>(); // 线性方程求解器
    //Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    auto solver_ptr = g2o::make_unique<Block>(std::move(linearSolver));

    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr ));

    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const cv::Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const cv::Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );
    std::cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<std::endl;

    std::cout<<std::endl<<"after optimization:"<<std::endl;
    std::cout<<"T="<<std::endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<std::endl;
}
