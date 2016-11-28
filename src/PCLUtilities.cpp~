#include <PCLUtilities.hpp>

using namespace pcl;

PCLUtilities::PCLUtilities()
{

}

PCLUtilities::PointCloud<PointXYZRGBA>::Ptr CloudDifference(PointCloud<PointXYZRGBA>::Ptr total, 
							    PointCloud<PointXYZRGBA>::Ptr sub)
{
  PointCloud<PointXYZRGBA> difference (new PointCloud<PointXYZRGBA>);

  ExtractIndices<PointXYZRGBA> extract;
  PointIndices<PointsXYZRGBA> inliers (new PointIndices());

  extract.setInputCloud(total);
  //extract.setIndices(inliers);
  extract.setNegative(true);
  //extract.filter(
  return difference;
}
