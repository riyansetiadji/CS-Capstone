#include <Octree.hpp>

using namespace pcl;

Octree::Octree(float res)
{
  resolution = res;
}

PointCloud<PointXYZRGBA>::Ptr
Octree::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  octree::OctreePointCloud<PointXYZRGBA> _octree (resolution);
  _octree.setInputCloud(cloud_in);
  _octree.addPointsFromInputCloud();

  octree::OctreePointCloud<PointXYZRGBA>::LeafNodeIterator itL(_octree); 

  /*std::vector<int> indexVector;
  unsigned int leafCount = 0;
  while (*++itL )
    {
      octree::OctreeLeafNode* node = *itL;
      }*/

  ReturnCloud = cloud_in;
  return ReturnCloud;
}

Eigen::Affine3f Octree::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void Octree::PrintAlgorithm()
{
  
}
