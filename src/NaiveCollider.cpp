#include <NaiveCollider.hpp>

using namespace pcl;

NaiveCollider::NaiveCollider(float epsilon)
{
  errorDistance = epsilon;
}

PointCloud<PointXYZRGBA>::Ptr
NaiveCollider::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud, 
		       const PointCloud<PointXYZRGBA>::Ptr &tCloud)
{
  int c1;
  int c2;
  unsigned int count = 0;
  for (c1 = 0; c1 < cloud->points.size(); c1++)
    {
      for (c2 = 0; c2 < tCloud->points.size(); c2++) 
	{
	  if (euclideanDistance(cloud->points[c1], tCloud->points[c2]) < errorDistance)
	    {
	      //cout << "touch at cloud point " << c1 << " and tCloud point " << c2 << endl;
	      ReturnCloud->push_back(cloud->points[c1]);
	    }
	}
    }
  return ReturnCloud;
}

