#ifndef HAND_ALGORITHM_H
#define HAND_ALGORITHM_H

#include <Algorithm.hpp>

class HandAlgorithm : Algorithm
{
public:
  Eigen::Affine3f handTransform;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr handCloud;
  pcl::PCA<pcl::PointXYZRGBA> handPCA;
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
};

#endif
