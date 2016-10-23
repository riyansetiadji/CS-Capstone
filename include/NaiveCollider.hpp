#ifndef NAIVE_COLLIDER_H
#define NAIVE_COLLIDER_H

#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <Algorithm.hpp>

using namespace pcl;

class NaiveCollider : public Algorithm
{
public:
  NaiveCollider(float ep);
float epsilonDistance;

  PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &,
	    const PointCloud<PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};

#endif
