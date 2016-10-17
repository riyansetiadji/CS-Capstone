#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <TrackingAlgorithm.hpp>

class CloudViewer : public TrackingAlgorithm
{
public: 
  CloudViewer();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};

#endif
