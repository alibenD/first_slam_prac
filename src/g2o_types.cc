/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: g2o_types.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-07 17:39:00
  * @last_modified_date: 2018-08-16 14:19:52
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <myslam/g2o_types.hh>

//CODE
namespace myslam
{
  void EdgeProjectXYZ2RGBD::computeError()
  {
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    _error = _measurement - pose->estimate().map(point->estimate());
  }

  void EdgeProjectXYZ2RGBD::linearizeOplus()
  {
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
    g2o::SE3Quat T(pose->estimate());
    g2o::VertexSBAPointXYZ* point = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz_ = point->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz_);
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    _jacobianOplusXi = -T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) = 0;
    _jacobianOplusXj(0,1) = -z;
    _jacobianOplusXj(0,2) = y;
    _jacobianOplusXj(0,3) = -1;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = 0;

    _jacobianOplusXj(1,0) = z;
    _jacobianOplusXj(1,1) = 0;
    _jacobianOplusXj(1,2) = -x;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1;
    _jacobianOplusXj(1,5) = 0;

    _jacobianOplusXj(2,0) = -y;
    _jacobianOplusXj(2,1) = x;
    _jacobianOplusXj(2,2) = 0;
    _jacobianOplusXj(2,3) = 0;
    _jacobianOplusXj(2,4) = 0;
    _jacobianOplusXj(2,5) = -1;
  }

  void EdgeProjectXYZ2RGBDPoseOnly::computeError()
  {
    const g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    _error = _measurement - pose->estimate().map(point_);
  }

  void EdgeProjectXYZ2RGBDPoseOnly::linearizeOplus()
  {
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    g2o::SE3Quat T(pose->estimate());
    Eigen::Vector3d xyz_trans = T.map(point_);
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi(0,0) = 0;
    _jacobianOplusXi(0,1) = -z;
    _jacobianOplusXi(0,2) = y;
    _jacobianOplusXi(0,3) = -1;
    _jacobianOplusXi(0,4) = 0;
    _jacobianOplusXi(0,5) = 0;

    _jacobianOplusXi(1,0) = z;
    _jacobianOplusXi(1,1) = 0;
    _jacobianOplusXi(1,2) = -x;
    _jacobianOplusXi(1,3) = 0;
    _jacobianOplusXi(1,4) = -1;
    _jacobianOplusXi(1,5) = 0;

    _jacobianOplusXi(2,0) = -y;
    _jacobianOplusXi(2,1) = x;
    _jacobianOplusXi(2,2) = 0;
    _jacobianOplusXi(2,3) = 0;
    _jacobianOplusXi(2,4) = 0;
    _jacobianOplusXi(2,5) = -1;
  }

  void EdgeProjectXYZ2UVPoseOnly::computeError()
  {
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0]);
    _error = _measurement - camera_->camera2pixel(pose->estimate().map(point_));
  }

  void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
  {
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Eigen::Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->get_fx();
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->get_fx();
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->get_fx();
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->get_fx();
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->get_fx();

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->get_fy();
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->get_fy();
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->get_fy();
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->get_fy();
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->get_fy();
  }
}   // END of namespace myslam
