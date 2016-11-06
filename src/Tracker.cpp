#include <Tracker.hpp>
#include <TrackerManager.hpp>
using namespace pcl;

Tracker::Tracker()
  {
    TargetTransform = Eigen::Affine3f::Identity ();
  }

Tracker::Tracker(std::string trackerName, char callback, 
		 float r, float g, float b, int ps, bool en, float h)
  {
    TargetTransform = Eigen::Affine3f::Identity ();
    TargetCloud.reset(new PointCloud<PointXYZRGBA>);

    callbackKey = callback;
    visualizerTextHeight = h;

    name = trackerName;
    red = r; green = g; blue = b;
    pointSize = ps;
    enabled = en;
    lastComputation = 0;

    boost::function<void(const visualization::KeyboardEvent &)> kb;
    kb = boost::bind (&Tracker::GetCallbackKey, this, _1);
    TrackerManager::GlobalTracker()->GetVisualizer()->registerKeyboardCallback(kb);
    UpdateVisualizer();
  }

//Calling this function effectively starts tracking by instantiating the thread
//We can add parameters to initialize Input and Collision Clouds if we need to make
//Track a parameterless function. Cannot have overloaded Collision version in that case
//And need to check for Collision cloud empty of make Collision Algorithm and type check
//Algorithm before execution
void Tracker::StartTracking()
{
  //We need to make Track parameterless, and then bind specific Tracker clouds as input
  //As we are already doing
  //trackerThread = new boost::thread(boost::bind (&TrackerManager::Track, this, _1));
}

PointCloud<PointXYZRGBA>::Ptr Tracker::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{ 
  TargetCloud.reset(new PointCloud<PointXYZRGBA>);
  if(enabled)
    {
      if(!cloud_in->points.empty())
	{
	  //boost::this_thread::sleep(boost::posix_time::milliseconds(milSeconds));
	  double start = getTime ();
	  TargetCloud = TrackerAlgorithm->Execute(cloud_in);
	  double end = getTime ();
	  lastComputation = (end - start)*1000;
	  UpdateVisualizer();
	}
    }
    return TargetCloud;
}

PointCloud<PointXYZRGBA>::Ptr Tracker::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_1,
					     const PointCloud<PointXYZRGBA>::Ptr &cloud_2)
{ 
  TargetCloud.reset(new PointCloud<PointXYZRGBA>);
  if(enabled)
    {
      //boost::this_thread::sleep(boost::posix_time::milliseconds(milSeconds));
      double start = getTime ();
      TargetCloud = TrackerAlgorithm->Execute(cloud_1, cloud_2);
      double end = getTime ();
      lastComputation = (end - start)*1000;
      UpdateVisualizer();
    }
    
    return TargetCloud;
}


void Tracker::GetCallbackKey(const visualization::KeyboardEvent& event)
{
  if (event.keyUp())
    {
      char key = event.getKeyCode();
      if(key == callbackKey)
        enabled = !enabled;
    }
}

void Tracker::UpdateVisualizer()
{
  
  visualization::PointCloudColorHandlerCustom<PointXYZRGBA> 
    colorHandler(TargetCloud, red, green, blue);
  boost::shared_ptr<visualization::PCLVisualizer> visualizer = 
    TrackerManager::GlobalTracker()->GetVisualizer();
  if (!visualizer->updatePointCloud (TargetCloud, colorHandler, name))
    {
      visualizer->addPointCloud (TargetCloud, colorHandler, name);
      visualizer->setPointCloudRenderingProperties
	(visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name);
    }
  PrintCloud();
}

//Need to add a smarter way to display text for each tracker automatically
void Tracker::PrintCloud()
{
  boost::shared_ptr<visualization::PCLVisualizer> visualizer = 
    TrackerManager::GlobalTracker()->GetVisualizer();
  std::string shapeString (1, callbackKey);
  visualizer->removeShape (shapeString);
  visualizer->addText ((boost::format ("%s tracking %d points at %f ms. Toggle key: %c") % name % TargetCloud->points.size () % lastComputation % callbackKey).str(), 10, visualizerTextHeight, 15, 1.0, 1.0, 1.0, shapeString);
}
