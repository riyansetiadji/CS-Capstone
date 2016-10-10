#ifndef TRACKER_H
#define TRACKER_H

#include <Algorithm.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

#define SLEEP_TIME 100
using namespace pcl;
class Tracker
{
  enum TargetState{ TARGET_UNKNOWN, TARGET_IDENTIFYING, TARGET_TRACKING };

protected:
  char callbackKey;
  float visualizerTextHeight;

 public:
  std::string name;
  bool enabled;
  int Target, pointSize;
  Eigen::Affine3f TargetTransform;
  Algorithm *TrackerAlgorithm;
  
  float red, green, blue; //Colors for visualization

  PointCloud<PointXYZRGBA>::Ptr TargetCloud;
  
  Tracker();
  Tracker(std::string, char, float, float, float, int, bool, float);
  void Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in);
  void GetCallbackKey(const visualization::KeyboardEvent &);
  
private:
  void UpdateVisualizer();
  void PrintCloud();
};

#endif
