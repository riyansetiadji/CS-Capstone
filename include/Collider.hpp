#ifndef COLLIDER_H
#define COLLIDER_H

#include <Container.hpp>
#include <ColliderAlgorithm.hpp>

using namespace pcl;

class Collider : public Container
{
public:
  ColliderAlgorithm *ColliderAlgorithm;
  
  Collider();
  Collider(std::string, char, float, float, float, int, bool, float);
  void Collide(const PointCloud<PointXYZRGBA>::Ptr &cloud1, 
	       const PointCloud<PointXYZRGBA>::Ptr &cloud2);
};

#endif
