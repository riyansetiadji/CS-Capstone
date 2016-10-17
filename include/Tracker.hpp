#ifndef TRACKER_H
#define TRACKER_H

#include <Container.hpp>
#include <TrackingAlgorithm.hpp>

using namespace pcl;

class Tracker : public Container
{
public:
  TrackingAlgorithm *TrackerAlgorithm;
  
  Tracker();
  Tracker(std::string, char, float, float, float, int, bool, float, 
	  boost::shared_ptr<visualization::PCLVisualizer>);
  void Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in);
  void GetCallbackKey(const visualization::KeyboardEvent &);

protected:
  char callbackKey;
  float visualizerTextHeight;

  void UpdateVisualizer();
  void PrintCloud();
};

#endif
