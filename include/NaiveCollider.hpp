#ifndef NAIVE_COLLIDER_H
#define NAIVE_COLLIDER_H

#include <CollisionAlgorithm.hpp>
#include <pcl/common/distances.h>

using namespace pcl;

class NaiveCollider : public CollisionAlgorithm
{
public:
  NaiveCollider(float epsilon);
  
  virtual PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &, const PointCloud<PointXYZRGBA>::Ptr &) = 0;
};

#endif
