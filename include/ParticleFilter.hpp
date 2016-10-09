#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Algorithm.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace pcl;

class ParticleFilter : public Algorithm
{
public:
  PointCloud<PointXYZRGBA>::Ptr cloud_pass_;
  PointCloud<PointXYZRGBA>::Ptr cloud_pass_downsampled_;
  PointCloud<PointXYZRGBA>::Ptr target_cloud;

  boost::mutex mtx_;
  boost::shared_ptr<tracking::ParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYZRPY> > tracker_;
  bool new_cloud_;
  double downsampling_grid_size_;
  int counter;
  std::string device_id;

  ParticleFilter(std::string, std::string);
  PointCloud<pcl::PointXYZRGBA>::Ptr 
  Execute(const PointCloud<PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();
  void PrintAlgorithm();

  void GridSampleApprox(const PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointCloud<PointXYZRGBA> &result, double leaf_size);
  void FilterPassThrough(const PointCloud<PointXYZRGBA>::Ptr &, PointCloud<PointXYZRGBA> &);
  bool DrawParticles();
  

};

#endif
