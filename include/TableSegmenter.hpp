#ifndef TABLE_SEGMENTER_H
#define TABLE_SEGMENTER_H

#include <Algorithm.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/centroid.h>

class TableSegmenter : public Algorithm
{
public:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projectionCloud;
  pcl::ModelCoefficients::Ptr tablePlaneCoefficient;

  TableSegmenter();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
  void TableFilter();
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();
};


#endif
