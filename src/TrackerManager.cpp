#include <TrackerManager.hpp>
#include <Tracker.hpp>

using namespace pcl;

  //Singleton class defines only one global pointer accessible by the type name
TrackerManager* TrackerManager::trackerInstance = 0;
TrackerManager* TrackerManager::GlobalTracker() 
{
  if(!trackerInstance)
    trackerInstance = new TrackerManager();  
  return trackerInstance;
}

TrackerManager::TrackerManager()
{
   zDepth = NEAR_DEPTH;
   showBackground = true;
   
   boost::function<void(const visualization::KeyboardEvent &)> kb;
   kb = boost::bind (&TrackerManager::InputManager, this, _1);
   visualizer.reset(new visualization::PCLVisualizer ("Tracking Viewer"));	
   visualizer->registerKeyboardCallback(kb);

   trackerHeight = 850;
   visualizer->setSize (1920, 1080);
   visualizer->addCoordinateSystem(0.1);
   visualizer->initCameraParameters();
}  

void TrackerManager::InitKinect()
{
  std::string device_id = "";
  kinectInterface = new OpenNIGrabber(device_id, OpenNIGrabber::OpenNI_QVGA_30Hz, OpenNIGrabber::OpenNI_QVGA_30Hz);

   boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> functionPointerNamedF;
   functionPointerNamedF = boost::bind (&TrackerManager::CloudGrabber, this, _1);
   boost::signals2::connection connect = 
     kinectInterface->registerCallback (functionPointerNamedF);
}

void TrackerManager::InputManager (const visualization::KeyboardEvent& event)
{
  if (event.keyUp())
    {
      char key = event.getKeyCode();
      if(key == 'b')
	showBackground = !showBackground;
    }
}

void TrackerManager::CloudGrabber(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  kinectCloud.reset(new PointCloud<PointXYZRGBA>);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      //kc->points.push_back(cloud_in->points[i]);
      kinectCloud->points.push_back(cloud_in->points[i]);
    }
}

//This function is registered to OpenNI, and begins calling the Tracking algorithms
void TrackerManager::ProcessingLoop(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  
}

//This function should step through the linked list every frame
//after processing, grabbing the finalized clouds from every
//tracker and rendering them
void TrackerManager::VisualizationLoop(bool showCloud)
{
  visualizer->spinOnce ();
  FPS_CALC ("visualization");
  if(showCloud)
  {
    std::string name = "Manager";
      visualization::PointCloudColorHandlerCustom<PointXYZRGBA>
	colorHandler(kinectCloud, 155, 155, 155);
if (!visualizer->updatePointCloud (kinectCloud, colorHandler, name))
    {

      visualizer->addPointCloud (kinectCloud, colorHandler, name);
      visualizer->setPointCloudRenderingProperties
	(visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
    }
  }
  visualizer->removeShape ("computation");
  visualizer->addText ((boost::format ("computation time per frame:     %f ms") % (computationTime)).str (), 10, 80, 15, 1.0, 1.0, 1.0, "computation");
}

/*void TrackerManager::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  OutputCloud.reset(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr temp_cloud (new PointCloud<PointXYZRGBA>);
  temp_cloud->height = CLOUD_HEIGHT;
  temp_cloud->width = CLOUD_WIDTH;
  
  PointXYZRGBA dummyPt;
  dummyPt.x = dummyPt.y = dummyPt.z = 0;
  dummyPt.r = dummyPt.g = dummyPt.b = 0;

  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      if (filter_mask[i])
	{
	  temp_cloud->points.push_back(cloud_in->points[i]);
	  OutputCloud->points.push_back(cloud_in->points[i]);
	}
      else
	temp_cloud->points.push_back(dummyPt);
    }
    }*/
