#ifndef OBJECT_ALGORITHM_H
#define OBJECT_ALGORITHM_H

#include <Algorithm.hpp>

class ObjectAlgorithm : Algorithm
{
public:
  pcl::PointCloud<pcl::PointXYZRGBA> ObjectCloud;
  void Execute();
};

#endif
