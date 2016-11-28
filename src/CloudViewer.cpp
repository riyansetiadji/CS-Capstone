#include <CloudViewer.hpp>

using namespace pcl;

CloudViewer::CloudViewer()
{

}

PointCloud<PointXYZRGBA>::Ptr
CloudViewer::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  ReturnCloud = cloud_in;
  return ReturnCloud;
}

Eigen::Affine3f CloudViewer::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void CloudViewer::PrintAlgorithm()
{
  
}
