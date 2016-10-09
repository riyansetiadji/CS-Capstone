#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <Algorithm.hpp>

class CloudViewer : public Algorithm
{
public: 
  CloudViewer();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};

#endif
