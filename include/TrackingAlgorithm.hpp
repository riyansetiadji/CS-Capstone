#ifndef TRACKING_ALGORITHM_H
#define TRACKING_ALGORITHM_H

#include <Algorithm.hpp>
#include <Eigen/Core>

using namespace pcl;

class TrackingAlgorithm : public Algorithm
{
public: 
  TrackingAlgorithm() {};
  
  virtual PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &) = 0;
  virtual Eigen::Affine3f ComputeTransform() = 0;
};

#endif
