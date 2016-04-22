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

//A singleton class holding important resources in the scene
class TrackerManager
{
 public:
  //Singleton class defines only one global pointer accessible by the type name
  static TrackerManager* GlobalTracker();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetKinectCloud()
  { return kinectCloud; }
  boost::shared_ptr<pcl::visualization::PCLVisualizer> GetVisualizer() 
  { return visualizer; }
  double GetZDepth() 
  { return zDepth; }

  void InputManager(const pcl::visualization::KeyboardEvent&);
  void ProcessingLoop(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in);
  void VisualizationLoop();  //Responsible for rendering kinect feed

private:
  static TrackerManager* trackerInstance;

  bool showBackground;
  double zDepth;
  double computationTime;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kinectCloud;
  void CloudGrabber(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud_in);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
  pcl::OpenNIGrabber* kinectInterface;

  TrackerManager()
{
   zDepth = NEAR_DEPTH;
   showBackground = true;

   std::string device_id = "";
   //kinectInterface = new OpenNIGrabber(device_id,
   // OpenNIGrabber::OpenNI_QVGA_30Hz, OpenNIGrabber::OpenNI_QVGA_30Hz);

   boost::function<void(const pcl::visualization::KeyboardEvent &)> kb;
   kb = boost::bind (&TrackerManager::InputManager, this, _1);
   visualizer.reset(new pcl::visualization::PCLVisualizer ("Tracking Viewer"));	
   visualizer->registerKeyboardCallback(kb);

   //boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> functionPointerNamedF;
   //functionPointerNamedF = boost::bind (&TrackerManager::CloudGrabber, this, _1);
   //boost::signals2::connection connect = 
   //  kinectInterface->registerCallback (functionPointerNamedF);
 
   //kinectInterface->start();
   
   //visualizer->getRenderWindow ()->SetSize ();
   visualizer->setSize (1280, 720);
   visualizer->addCoordinateSystem(0.1);
   visualizer->initCameraParameters();
}  
};

#endif
