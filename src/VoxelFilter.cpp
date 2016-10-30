#include<VoxelFilter.hpp>

using namespace pcl;

VoxelFilter::VoxelFilter(float x, float y, float z)
{
  leafX = x;
  leafY = y;
  leafZ = z;
}

PointCloud<PointXYZRGBA>::Ptr
VoxelFilter::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  ReturnCloud.reset(new PointCloud<PointXYZRGBA>);
  VoxelGrid<PointXYZRGBA> downsampler;
  downsampler.setInputCloud (cloud_in);
  downsampler.setLeafSize (leafX, leafY, leafZ);
  downsampler.filter (*ReturnCloud);
  return ReturnCloud;
}

Eigen::Affine3f VoxelFilter::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void VoxelFilter::PrintAlgorithm()
{
  
}
