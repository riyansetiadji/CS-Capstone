#ifndef TRACKER_H
#define TRACKER_H

#include <Algorithm.hpp>

class Tracker
{
  enum TargetState{ TARGET_UNKNOWN, TARGET_IDENTIFYING, TARGET_TRACKING };

 public:
  std::string name, shape;
  int Target, pointSize;
  Eigen::Affine3f TargetTransform;
  Algorithm *TrackerAlgorithm;
  Tracker *NextTracker;
  float red, green, blue; //Colors for visualization

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr TargetCloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutputCloud;
  
  Tracker();
  Tracker(std::string, std::string, float, float, float, int);
  void Track(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in);
  void KeyboardCallback();  
};

#endif
