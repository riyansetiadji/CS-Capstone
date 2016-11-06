#ifndef TRACKER_H
#define TRACKER_H

#include <Algorithm.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <string>

#define SLEEP_TIME 100
using namespace pcl;
class Tracker
{

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
  float lastComputation;

  int loopCounter;

  boost::thread* trackerThread;
  float threadLatency;
  Tracker* targetTracker;
  PointCloud<PointXYZRGBA>::Ptr InputCloud;
  PointCloud<PointXYZRGBA>::Ptr ColliderCloud;
  PointCloud<PointXYZRGBA>::Ptr TargetCloud;
  
  Tracker();
  Tracker(std::string, char, float, float, float, int, bool, float);
  
  void SetLoopCounter(int);
  void SetThreadLatency(float);
  void StartTracking();
  void ThreadedTrack();
  void ThreadedCollision();

  PointCloud<PointXYZRGBA>::Ptr Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in);
  PointCloud<PointXYZRGBA>::Ptr Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_1,
				      const PointCloud<PointXYZRGBA>::Ptr &cloud_2);

  void GetCallbackKey(const visualization::KeyboardEvent &);
  
private:
  void UpdateVisualizer();
  void PrintCloud();
};

#endif
