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
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)


class TrackerManager : public Tracker
{
 public:
  bool showBackground;
  double zDepth;
  double computationTime;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
  pcl::OpenNIGrabber* kinectInterface;

  bool filter_mask[CLOUD_WIDTH * CLOUD_HEIGHT];

  TrackerManager();
  void Track(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in);
  void KeyboardCallback();

  void InputManager(const pcl::visualization::KeyboardEvent&);
  void ProcessingLoop(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in);
  void VisualizationLoop();  //Responsible for rendering kinect feed
  void UpdateTracker(Tracker*);
  void PrintCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, 
		  std::string, std::string);

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  FilterInput(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &);
};

#endif
