#ifndef CONTAINER_H
#define CONTAINER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using namespace pcl;
class Container
{
 public:
  std::string name;
  bool enabled;
  int pointSize;
  float red, green, blue; //Colors for visualization

  PointCloud<PointXYZRGBA>::Ptr TargetCloud;
  
  Container();
  Container(std::string trackerName, char callback, 
	    float r, float g, float b, int ps, bool en, float h);
  void GetCallbackKey(const visualization::KeyboardEvent &);
 
protected:
  char callbackKey;
  float visualizerTextHeight;

  void UpdateVisualizer();
  void PrintCloud();
};

#endif
