#ifndef COLLISION_ALGORITHM_H
#define COLLISION_ALGORITHM_H

#include <Algorithm.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/console/print.h>

#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

using namespace pcl;

class CollisionAlgorithm : public Algorithm
{
public: 
  float errorDistance;

  CollisionAlgorithm() {};
  
  virtual PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &, const PointCloud<PointXYZRGBA>::Ptr &) = 0;
};

#endif
