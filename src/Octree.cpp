#include <Octree.hpp>

using namespace pcl;

Octree::Octree(float res)
{
  resolution = res;
}

PointCloud<PointXYZRGBA>::Ptr
Octree::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud1,
		const PointCloud<PointXYZRGBA>::Ptr &cloud2)
{
  octree::OctreePointCloud<PointXYZRGBA> _octree (resolution);
  _octree.setInputCloud(cloud1);
  _octree.addPointsFromInputCloud();

  //octree::OctreePointCloud<PointXYZRGBA>::LeafNodeIterator itL(_octree); 
  //octree::OctreePointCloud<PointXYZRGBA>::DepthFirstIterator itL(_octree);

  /*std::vector<int> indexVector;
  unsigned int leafCount = 0;
  while (*++itL )
    {
      octree::OctreeLeafNode* node = *itL;
      }*/

  ReturnCloud = cloud1;
  return ReturnCloud;
}

Eigen::Affine3f Octree::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void Octree::PrintAlgorithm()
{
  
}
