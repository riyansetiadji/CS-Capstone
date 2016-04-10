#include <TrackerManager.hpp>
#include <Tracker.hpp>

using namespace pcl;

TrackerManager::TrackerManager()
{
   zDepth = NEAR_DEPTH;
   name = "BackgroundFilter";
   shape = "B";
   red = 75;
   green = 75;
   blue = 75;
   pointSize = 8;

   showBackground = true;

   std::string device_id = "";
   kinectInterface = new OpenNIGrabber(device_id,
    OpenNIGrabber::OpenNI_QVGA_30Hz, OpenNIGrabber::OpenNI_QVGA_30Hz);

   boost::function<void(const visualization::KeyboardEvent &)> kb;
   kb = boost::bind (&TrackerManager::InputManager, this, _1);
   visualizer.reset(new visualization::PCLVisualizer ("Tracking Viewer"));	
   visualizer->registerKeyboardCallback(kb);

   boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> functionPointerNamedF;
   functionPointerNamedF = boost::bind (&TrackerManager::ProcessingLoop, this, _1);
   boost::signals2::connection connect = 
     kinectInterface->registerCallback (functionPointerNamedF);
 
   kinectInterface->start();
   
   //visualizer->getRenderWindow ()->SetSize ();
   visualizer->setSize (1280, 720);
   visualizer->addCoordinateSystem(0.1);
   visualizer->initCameraParameters();
}

//This function is registered to OpenNI, and begins calling the Tracking algorithms
void TrackerManager::ProcessingLoop(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  double start = getTime ();
  PointCloud<PointXYZRGBA>::Ptr kinectInput = FilterInput(cloud_in);
  Track(kinectInput);
  Tracker *CurrentTracker = NextTracker;
  while(CurrentTracker)
    {
      //cout << ("Start Tracker: " + CurrentTracker->name + "\n");
      CurrentTracker->Track(this->OutputCloud); 
      CurrentTracker = CurrentTracker->NextTracker;
    }
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
				
  Tracker *CurrentTracker = this;
  if (showBackground)
  {
    UpdateTracker(CurrentTracker);
    //visualizer->addCoordinateSystem(0.1, CurrentTracker->TargetTransform, 
    //CurrentTracker->name);
    //visualizer->resetCameraViewpoint (CurrentTracker->name);
  }

  CurrentTracker = CurrentTracker->NextTracker;
  while(CurrentTracker)
    {
      UpdateTracker(CurrentTracker);
      //visualizer->resetCameraViewpoint(CurrentTracker->name);
      CurrentTracker = CurrentTracker->NextTracker;
      }

  visualizer->removeShape ("computation");
  visualizer->addText ((boost::format ("computation time per frame:     %f ms") % (computationTime)).str (), 10, 80, 15, 1.0, 1.0, 1.0, "computation");
}

PointCloud<PointXYZRGBA>::Ptr 
TrackerManager::FilterInput(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
    TargetCloud.reset(new PointCloud<PointXYZRGBA>);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      if (( cloud_in->points[i].z < zDepth) && 
	  ( cloud_in->points[i].x < 0.5))
        {
	  TargetCloud->points.push_back(cloud_in->points[i]);
	  filter_mask[i] = true;
        }
      else
        filter_mask[i] = false;
    }
  return TargetCloud;
}

void TrackerManager::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
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
}


void TrackerManager::UpdateTracker(Tracker *tracker)
{
  visualization::PointCloudColorHandlerCustom<PointXYZRGBA> 
    colorHandler(tracker->TargetCloud, tracker->red, 
		 tracker->green, tracker->blue);
  if (!visualizer->updatePointCloud (tracker->TargetCloud, 
				     colorHandler, tracker->name))
    {
      visualizer->addPointCloud (tracker->TargetCloud, 
				 colorHandler, tracker->name);
      visualizer->setPointCloudRenderingProperties
	(visualization::PCL_VISUALIZER_POINT_SIZE, 
	 tracker->pointSize, tracker->name);
    }
  //PrintCloud(tracker->TargetCloud, 
  //	     tracker->name + " tracking %d points", tracker->shape);
}

void TrackerManager::PrintCloud(PointCloud<PointXYZRGBA>::Ptr cloud, 
				std::string message, std::string shape)
{					
  visualizer->removeShape (shape);
  visualizer->addText ((boost::format (message) % cloud->points.size ()).str (), 10, 40, 15, 1.0, 1.0, 1.0, shape);
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

void TrackerManager::KeyboardCallback ()
{
  
}
