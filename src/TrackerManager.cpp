#include <TrackerManager.hpp>
#include <Tracker.hpp>

using namespace pcl;

  //Singleton class defines only one global pointer accessible by the type name
TrackerManager* TrackerManager::trackerInstance = 0;
TrackerManager* TrackerManager::GlobalTracker() 
{
  if(!trackerInstance)
    trackerInstance = new TrackerManager;  
  return trackerInstance;
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

//This function is registered to OpenNI, and begins calling the Tracking algorithms
void TrackerManager::ProcessingLoop(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  double start = getTime ();

  double end = getTime ();
  computationTime = (end - start)*1000;
}

//This function should step through the linked list every frame
//after processing, grabbing the finalized clouds from every
//tracker and rendering them
void TrackerManager::VisualizationLoop()
{
  visualizer->spinOnce ();
  FPS_CALC ("visualization");

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
