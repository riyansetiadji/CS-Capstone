#ifndef KD_TRACKER_H
#define KD_TRACKER_H

#include <Algorithm.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath> 

using namespace pcl;

class KDTracker : public Algorithm
{
public:
  int last_frame_hand_points;
  int last_frame_object_points;
  double hand_h;
  double hand_s;
  double hand_v;
  double object_h;
  double object_s;
  double object_v;
  float distance_threshold;
  float point_color_threshold;
  float region_color_threshold;
  int min_cluster_size;
  int max_cluster_size;
  float threshold;

  PointCloud<PointXYZRGBA>::Ptr hand_cloud;
  PointCloud<PointXYZRGBA>::Ptr hand_object_cloud;
  PointCloud<PointXYZRGBA>::Ptr hand_subtraction;
  
  search::Search <PointXYZRGBA>::Ptr tree;
  
  KDTracker(std::string oPath, std::string hPath);
  PointCloud<PointXYZRGBA>::Ptr 
  Execute(const PointCloud<PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
std::vector <PointIndices> getClusters(const PointCloud<PointXYZRGBA>::Ptr &cloud);
  void RGBToHSV(float r, float g, float b, float *h, float *s, float *v);
  void getAvgHSV(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double *h_avg, double *s_avg, double *v_avg, int showcld);
  void getAvgHSVForOnePoints(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double *h_avg, double *s_avg, double *v_avg,int showcld);
  void getObjectAvgHSV(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double hue,double *objectH, double *objectS,double *objectV,int showcld);
  PointCloud<PointXYZRGBA>::Ptr PointFilter(const PointCloud<PointXYZRGBA>::Ptr &cloud,double handH, double objectH);
 PointCloud<PointXYZRGBA>::Ptr filter_table(const PointCloud<PointXYZRGBA>::ConstPtr &cloud);
  PointCloud<PointXYZRGBA>::Ptr filterSetOfPcdWithClusters(const PointCloud<PointXYZRGBA>::Ptr &cloud,double handH,double objectH,int showcld);
  PointCloud<PointXYZRGBA>::Ptr filterSetOfPcdWithPoints(const PointCloud<PointXYZRGBA>::Ptr &cloud,double handH,double objectH,int showcld);

  PointCloud<PointXYZRGBA>::Ptr getObjectCloud();
};

#endif
