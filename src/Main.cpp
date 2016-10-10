#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>
#include <KDTracker.hpp>
#include <ParticleFilter.hpp>
#include <CloudViewer.hpp>
#include <PCDInterface.hpp>

#include <string>
#include <stdlib.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
  cout << "Eat Your Greens\n";
  int kinectInput = std::atoi(argv[1]);
  cout << kinectInput << std::endl;
  CloudViewer* cViewer = new CloudViewer();  
  Tracker* TrackViewer = TrackerManager::GlobalTracker()->
    CreateTracker(cViewer, "CloudViewer", 'z', 125, 175, 215, 1, true);

  KDTracker* kdTracker = new KDTracker("../data/andy_hand.pcd", "../data/chewy.pcd");
  Tracker* TrackStar = TrackerManager::GlobalTracker()->
    CreateTracker(kdTracker, "KDTracker", 'a', 9, 251, 2, 4, false);
 
  std::string file = "../data/chewy.pcd";
  std::string did = "666";
  ParticleFilter* pf = new ParticleFilter(file, did);
  Tracker* TrackMarks = TrackerManager::GlobalTracker()->
    CreateTracker(pf, "ParticleFilter", 'b', 9, 0, 255, 2, false);
  
  PCDInterface* pcdInterface = new PCDInterface(argv[2][0], std::atoi(argv[3]));
  float milSeconds = std::atof(argv[4]);
  srand(time(NULL));

//Need a way to register trackers with the TrackerManager so they will respond to//the keyboard callback.
  if(kinectInput)
    {
      TrackerManager::GlobalTracker()->InitKinect();
      if(pcdInterface->Write())
	{
	  TrackerManager::GlobalTracker()->GetOpenNIGrabber()->start();
	  while(TrackerManager::GlobalTracker()->GetOpenNIGrabber()->isRunning())
	  {
	    std::cout << "Writing" << std::endl;
	    TrackerManager::GlobalTracker()->VisualizationLoop(true);
	  }
	}
      else
	{
	  TrackerManager::GlobalTracker()->GetOpenNIGrabber()->start();
        while(TrackerManager::GlobalTracker()->GetOpenNIGrabber()->isRunning())
	  {
	    TrackerManager::GlobalTracker()->VisualizationLoop(true);
	    TrackMarks->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	    //TrackStar->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	  }
	}
    }
  else
    {
      cout << "At the files\n";
	  while(pcdInterface->fileIndex < pcdInterface->maxFiles)
	    {
	      TrackerManager::GlobalTracker()->GetVisualizer()->spinOnce();
	      TrackViewer->Track(pcdInterface->GetNextCloud());
	      TrackStar->Track(pcdInterface->GetNextCloud());
	      TrackMarks->Track(pcdInterface->GetNextCloud());
	      boost::this_thread::sleep(boost::posix_time::milliseconds(milSeconds));
	    }
    }

  return 0;
}

