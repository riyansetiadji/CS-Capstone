#include <Tracker.hpp>
#include <TrackerManager.hpp>
using namespace pcl;

Tracker::Tracker()
  {
    Target = TARGET_UNKNOWN;
    TargetTransform = Eigen::Affine3f::Identity ();
  }

Tracker::Tracker(std::string trackerName, char callback, 
		 float r, float g, float b, int ps, bool en, float h)
  {
    Target = TARGET_UNKNOWN;
    TargetTransform = Eigen::Affine3f::Identity ();

    callbackKey = callback;
    visualizerTextHeight = h;

    name = trackerName;
    red = r; green = g; blue = b;
    pointSize = ps;
    enabled = en;

    boost::function<void(const visualization::KeyboardEvent &)> kb;
    kb = boost::bind (&Tracker::GetCallbackKey, this, _1);
    TrackerManager::GlobalTracker()->GetVisualizer()->registerKeyboardCallback(kb);
  }

void Tracker::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{ 
  TargetCloud.reset(new PointCloud<PointXYZRGBA>);
  if(enabled)
    {
      if(!cloud_in->points.empty())
	{
	  if(Target == TARGET_UNKNOWN)
	    {
	      TargetCloud = TrackerAlgorithm->Execute(cloud_in);
	      //TrackerAlgorithm->ComputeTransform();
	      //Target = TARGET_TRACKING;
	    }
	  else if(Target == TARGET_IDENTIFYING)
	    {
	      //TrackerAlgorithm->ComputeTransform();
	    }
	  else
	    {
	      //TrackerAlgorithm->ComputeTransform();
	    }
	}
    }
      UpdateVisualizer();
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
  visualizer->addText ((boost::format (name + " tracking %d points. Toggle key: " + callbackKey) % TargetCloud->points.size ()).str (), 10, visualizerTextHeight, 15, 1.0, 1.0, 1.0, shapeString);
}
