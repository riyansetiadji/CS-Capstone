#ifndef PCL_UTILITIES
#define PCL_UTILITIES

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace pcl;

class PCLUtilities
{
public:
  PCLUtilities();
  PointCloud<PointXYZRGBA>::Ptr CloudDifference(PointCloud<PointXYZRGBA>::Ptr total, 
						PointCloud<PointXYZRGBA>::Ptr sub);
};

#endif
