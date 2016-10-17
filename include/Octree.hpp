#ifndef OCTREE_H
#define OCTREE_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_pointcloud.h>

#include <iostream>
#include <vector>
#include <ctime>

#include <CollisionAlgorithm.hpp>

using namespace pcl;

class Octree : public CollisionAlgorithm
{
public:
  Octree(float res);
  float resolution;

  PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &, const PointCloud<PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};

#endif
