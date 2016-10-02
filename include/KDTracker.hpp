#ifndef KD_TRACKER_H
#define KD_TRACKER_H

#include <Algorithm.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <boost/make_shared.hpp>

class KDTracker : public Algorithm
{
public:
  
  float hgoal;  //I'm guessing a scanning routine would check
  float sgoal; //skin colors to dynamically get these goals
  float vgoal;
  float threshold; //a percentage from 0 to 1 

  pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree;
  
  KDTracker();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();

  void RGBToHSV(float r, float g, float b, float *h, float *s, float *v);
};

#endif
