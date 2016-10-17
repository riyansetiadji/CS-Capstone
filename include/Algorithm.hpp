#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/console/print.h>

#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

using namespace pcl;

class Algorithm
{
public: 
  PointCloud<PointXYZRGBA>::Ptr ReturnCloud;
  Algorithm() {};
  virtual void PrintAlgorithm() = 0;
};

#endif
