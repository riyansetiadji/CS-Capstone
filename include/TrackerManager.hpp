#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

#include <Tracker.hpp>

#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/console/time.h>
#include <pcl/console/parse.h>

#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>

using namespace pcl;

#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

#define NEAR_DEPTH 1.0
#define MEDIUM_DEPTH 2.0
#define FAR_DEPTH 4.0

#define BACKGROUND_TOGGLE 'b'

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

//A singleton class holding important resources in the scene
class TrackerManager
{
 public:
  TrackerManager();
  //Singleton class defines only one global pointer accessible by the type name
  static TrackerManager* GlobalTracker();
  PointCloud<PointXYZRGBA>::Ptr GetKinectCloud()
  { return kinectCloud; }
  OpenNIGrabber* GetOpenNIGrabber()
  { return kinectInterface; }
  boost::shared_ptr<visualization::PCLVisualizer> GetVisualizer() 
  { return visualizer; }
  double GetZDepth() 
  { return zDepth; }

  void CloudGrabber(const PointCloud<PointXYZRGBA>::ConstPtr & cloud_in);
  void InputManager(const visualization::KeyboardEvent&);
  void ProcessingLoop(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in);
  void VisualizationLoop(bool);  //Responsible for rendering kinect feed

private:
  static TrackerManager* trackerInstance;

  bool showBackground;
  double zDepth;
  double computationTime;

  PointCloud<PointXYZRGBA>::Ptr kinectCloud;
  
  boost::shared_ptr<visualization::PCLVisualizer> visualizer;
  OpenNIGrabber* kinectInterface;

};

#endif
