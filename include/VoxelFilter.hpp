#ifndef VOXEL_FILTER_H
#define VOXEL_FILTER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <Algorithm.hpp>

using namespace pcl;

class VoxelFilter : public Algorithm
{
public: 
float leafX, leafY, leafZ;
VoxelFilter(float, float, float);

  PointCloud<PointXYZRGBA>::Ptr
  Execute(const PointCloud<PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};

#endif
