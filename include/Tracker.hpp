#ifndef TRACKER_H
#define TRACKER_H

#include <Algorithm.hpp>

#include <string>

#define SLEEP_TIME 100
class Tracker
{
  enum TargetState{ TARGET_UNKNOWN, TARGET_IDENTIFYING, TARGET_TRACKING };

protected:
  char callbackKey;

 public:
  std::string name;
  bool enabled;
  int Target, pointSize;
  Eigen::Affine3f TargetTransform;
  Algorithm *TrackerAlgorithm;
  
  float red, green, blue; //Colors for visualization

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr TargetCloud;
  
  Tracker();
  Tracker(std::string, char, float, float, float, int);
  void Track(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in);
  char GetCallbackKey() { return callbackKey; }
  
private:
  void UpdateVisualizer();
  void PrintCloud();
};

#endif
