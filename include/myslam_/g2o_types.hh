#ifndef __G2O_TYPES_HH__
#define __G2O_TYPES_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: g2o_types.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-07 17:39:00
  * @last_modified_date: 2018-08-16 14:08:18
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <myslam/common.hh>
#include <myslam/camera.hh>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

// Declaration
namespace myslam
{
  class EdgeProjectXYZ2RGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual void computeError();
      virtual void linearizeOplus();
      virtual bool read(std::istream& in){ return true; }
      virtual bool write(std::ostream& out) const { return true; }
  };

  class EdgeProjectXYZ2RGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual void computeError();
      virtual void linearizeOplus();
      virtual bool read(std::istream& in) { return true; }
      virtual bool write(std::ostream& out) const { return true; }

      Eigen::Vector3d point_;
  };

  class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      virtual void computeError();
      virtual void linearizeOplus();
      virtual bool read(std::istream& in) { return true; }
      virtual bool write(std::ostream& os) const { return true; }

      Eigen::Vector3d point_;
      Camera* camera_;
  };
}   // END od namespace myslam

#endif // __G2O_TYPES_HH__
