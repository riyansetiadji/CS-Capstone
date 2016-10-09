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
  int pcdInput = std::atoi(argv[1]);
  
  Tracker* TrackViewer = new Tracker("View the cloud, be the cloud", 'z', 125, 175, 215, 5);
  CloudViewer* cViewer = new CloudViewer();
  TrackViewer->TrackerAlgorithm = cViewer;

  Tracker* TrackStar = new Tracker("Random Ass Tracker", 'a', 9, 0, 255, 4);
  KDTracker* kdTracker = new KDTracker("../data/andy_hand.pcd", "../data/chewy.pcd");
  TrackStar->TrackerAlgorithm = kdTracker;

  Tracker* TrackMarks = new Tracker("Strung out Tracker", 'b', 9, 0, 255, 2);
  std::string file = "../data/bbq_pringles.pcd";
  std::string did = "666";
  ParticleFilter* pf = new ParticleFilter(file, did);
  TrackMarks->TrackerAlgorithm = pf;

  //Tracker* TrackAndField = new Tracker("Trackin'", 'c', 4, 255, 2, 255);
  
  //RandomCloud* randomCloud = new RandomCloud();
  //RandomAssTracker->TrackerAlgorithm = randomCloud;

  srand(time(NULL));

//Need a way to register trackers with the TrackerManager so they will respond to//the keyboard callback.
  if(pcdInput)
    {
      PCDInterface* pcdInterface = new PCDInterface(argv[2][0], std::atoi(argv[3]));
      if(pcdInterface->Write())
	{
	  while(TrackerManager::GlobalTracker()->GetOpenNIGrabber()->isRunning())
	  {
	    std::cout << "Writing" << std::endl;
	    TrackerManager::GlobalTracker()->VisualizationLoop(true);
	  }
	}
      else
	{
	  TrackerManager::GlobalTracker()->GetOpenNIGrabber()->stop();
	  while(pcdInterface->fileIndex < pcdInterface->maxFiles)
	    {
	      TrackerManager::GlobalTracker()->GetVisualizer()->spinOnce();
	      //TrackViewer->Track(pcdInterface->GetNextCloud());
	      //TrackStar->Track(pcdInterface->GetNextCloud());
	      TrackMarks->Track(pcdInterface->GetNextCloud());
	      boost::this_thread::sleep(boost::posix_time::seconds(0.5));
	    }
	}
    }
  else
    {
        while(!TrackerManager::GlobalTracker()->GetVisualizer()->wasStopped())
	  {
	    TrackerManager::GlobalTracker()->VisualizationLoop(true);
	    TrackMarks->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	    //TrackStar->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	  }
    }

  return 0;
}

