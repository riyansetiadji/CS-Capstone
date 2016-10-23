#include <NaiveCollider.hpp>

using namespace pcl;

NaiveCollider::NaiveCollider(float epDis)
{
  epsilonDistance = epDis;
}

PointCloud<PointXYZRGBA>::Ptr
NaiveCollider::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_1, 
		       const PointCloud<PointXYZRGBA>::Ptr &cloud_2)
{
  ReturnCloud.reset(new PointCloud<PointXYZRGBA>);
  int c1;
  int c2;
  unsigned int count = 0;
  for (c1 = 0; c1 < cloud_1->points.size(); c1++)
    {
      for (c2 = 0; c2 < cloud_2->points.size(); c2++) 
	{
	  if (euclideanDistance(cloud_1->points[c1], cloud_2->points[c2]) < 0.01f)
	    {
	      ReturnCloud->points.push_back(cloud_1->points[c1]);
	    }
	}
    }
  return ReturnCloud;
}

Eigen::Affine3f NaiveCollider::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void NaiveCollider::PrintAlgorithm()
{
  
}
