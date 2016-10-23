#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>
#include <KDTracker.hpp>
#include <ParticleFilter.hpp>
#include <NaiveCollider.hpp>
#include <CloudViewer.hpp>
#include <PCDInterface.hpp>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
  cout << "Eat Your Greens\n";
  int kinectInput = std::atoi(argv[1]);
   std::string fileloc = argv[5];
  cout << kinectInput << std::endl;
  
  CloudViewer* cViewer = new CloudViewer();  
  Tracker* TrackViewer = TrackerManager::GlobalTracker()->
    CreateTracker(cViewer, "CloudViewer", 'z', 125, 175, 215, 1, true);
  Tracker* ObjectViewer = TrackerManager::GlobalTracker()->
    CreateTracker(cViewer, "ObjectViewer", 'y', 125, 185, 15, 2, false);

  KDTracker* kdTracker = new KDTracker("../data/andy_hand.pcd", "../data/"+fileloc+"/"+fileloc+"10.pcd");
  Tracker* TrackHandFilter = TrackerManager::GlobalTracker()->
    CreateTracker(kdTracker, "KDTracker", 'a', 9, 251, 2, 4, false);
 
  std::string file = "../data/bbq_pringles.pcd";
  std::string did = "666";
  ParticleFilter* pf = new ParticleFilter(file, did);
  Tracker* TrackParticleFilter = TrackerManager::GlobalTracker()->
    CreateTracker(pf, "ParticleFilter", 'b', 9, 0, 255, 2, false);

  NaiveCollider* nc = new NaiveCollider(1.1f);
  Tracker* TrackNaiveCollision = TrackerManager::GlobalTracker()->
    CreateTracker(nc, "NaiveCollision", 'c', 255, 0, 15, 5, false);
 
  PCDInterface* pcdInterface = new PCDInterface(argv[2][0], std::atoi(argv[3]),fileloc);
  float milSeconds = std::atof(argv[4]);
  srand(time(NULL));

//Need a way to register trackers with the TrackerManager so they will respond to//the keyboard callback.
  if(kinectInput)
    {
      TrackerManager::GlobalTracker()->InitKinect();
      if(pcdInterface->Write())
	{
   // std::cout << "passed write function" << std::endl;
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
	    TrackParticleFilter->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	    TrackHandFilter->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	  }
	}
    }
  else
    {
      //cout << "At the files\n";
	  while(pcdInterface->fileIndex < pcdInterface->maxFiles)
	    {
	      TrackerManager::GlobalTracker()->GetVisualizer()->spinOnce();
	     TrackViewer->Track(pcdInterface->GetNextCloud());
	      PointCloud<PointXYZRGBA>::Ptr handCloud = 
		TrackHandFilter->Track(pcdInterface->GetNextCloud());
	      ObjectViewer->Track(kdTracker->getObjectCloud());
	      PointCloud<PointXYZRGBA>::Ptr particleCloud = 
		TrackParticleFilter->Track(kdTracker->getObjectCloud());
	      PointCloud<PointXYZRGBA>::Ptr collideCloud =
	      TrackNaiveCollision->Track(particleCloud, handCloud);
	      boost::this_thread::sleep(boost::posix_time::milliseconds(milSeconds));
	    }
    }

  return 0;
}

