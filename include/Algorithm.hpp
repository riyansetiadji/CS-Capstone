#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/console/print.h>

#include <Eigen/Core>

#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

class Algorithm
{
public: 
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ReturnCloud;
  
  Algorithm() {};
  
  virtual pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &) = 0;
  virtual Eigen::Affine3f ComputeTransform() = 0;
  virtual void PrintAlgorithm() = 0;
};

#endif
