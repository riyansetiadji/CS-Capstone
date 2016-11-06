#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>
#include <KDTracker.hpp>
#include <ParticleFilter.hpp>
#include <NaiveCollider.hpp>
#include <CloudViewer.hpp>
#include <VoxelFilter.hpp>
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

  VoxelFilter* vFilter = new VoxelFilter(0.009, 0.009, 0.009);
  Tracker* TrackVoxelFilter = TrackerManager::GlobalTracker()->
    CreateTracker(vFilter, "VoxelTracker", 'v', 5, 225, 195, 3, true);

  KDTracker* kdTracker = new KDTracker("../data/andy_hand.pcd", "../data/"+fileloc+"/"+fileloc+"10.pcd");
  Tracker* TrackHandFilter = TrackerManager::GlobalTracker()->
    CreateTracker(kdTracker, "KDTracker", 'a', 9, 251, 2, 5, false);
 
  std::string file = "../data/bbq.pcd";
  std::string did = "666";
  ParticleFilter* pf = new ParticleFilter(file, did);
  Tracker* TrackParticleFilter = TrackerManager::GlobalTracker()->
    CreateTracker(pf, "ParticleFilter", 'b', 9, 0, 255, 5, false);

  NaiveCollider* nc = new NaiveCollider(1.1f);
  Tracker* TrackNaiveCollision = TrackerManager::GlobalTracker()->
    CreateTracker(nc, "NaiveCollision", 'c', 255, 0, 15, 15, false);
 
  int trackerSize = 3;
  //Tracker* trackers[5] = {TrackViewer, TrackVoxelFilter, TrackHandFilter, TrackParticleFilter, TrackNaiveCollision};
  Tracker* trackers[3] = {TrackVoxelFilter, TrackHandFilter, TrackParticleFilter};
  PCDInterface* pcdInterface = new PCDInterface(argv[2][0], std::atoi(argv[3]),fileloc);
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
	    TrackParticleFilter->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	    TrackHandFilter->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	  }
	}
    }
  else
    {
	      
	      //We may need to thread PCDInterface in order to have a constant reference
	        
	      //TrackViewer->SetThreadLatency(100);
	      TrackViewer->SetLoopCounter(1);
	      
	      //TrackVoxelFilter->InputCloud = TrackViewer->TargetCloud;
	      TrackVoxelFilter->targetTracker = TrackViewer;
	      //TrackVoxelFilter->SetThreadLatency(100);
	      TrackVoxelFilter->SetLoopCounter(1);
	      
	      //TrackHandFilter->InputCloud = TrackVoxelFilter->TargetCloud;
	      TrackHandFilter->targetTracker = TrackVoxelFilter;
	      //TrackHandFilter->SetThreadLatency(100);
	      TrackHandFilter->SetLoopCounter(1);
	      
	      //TrackParticleFilter->InputCloud = TrackHandFilter->TargetCloud;
	      TrackParticleFilter->targetTracker = TrackHandFilter;
	      //TrackParticleFilter->SetThreadLatency(100);
	      TrackParticleFilter->SetLoopCounter(1);
	      
	      //TrackNaiveCollision->InputCloud = kdTracker->getObjectCloud(); //This probably won't work either
	      //TrackNaiveCollision->ColliderCloud = TrackHandFilter->TargetCloud;
	      //TrackNaiveCollision->SetThreadLatency(100);
	      TrackNaiveCollision->SetLoopCounter(1);
	      
	      //TrackViewer->StartTracking();
	      //TrackVoxelFilter->StartTracking();
	      //TrackHandFilter->StartTracking();
	      //TrackParticleFilter->StartTracking();
	      //TrackNaiveCollision->StartTracking();

	      /*PointCloud<PointXYZRGBA>::Ptr kinectCloud =
	      TrackViewer->Track(pcdInterface->GetNextCloud());
	     PointCloud<PointXYZRGBA>::Ptr filterCloud = 
	       TrackVoxelFilter->Track(kinectCloud);
	     PointCloud<PointXYZRGBA>::Ptr handCloud = 
		TrackHandFilter->Track(filterCloud);
	      ObjectViewer->Track(kdTracker->getObjectCloud());
	      PointCloud<PointXYZRGBA>::Ptr particleCloud = 
		TrackParticleFilter->Track(kdTracker->getObjectCloud());
	      PointCloud<PointXYZRGBA>::Ptr collideCloud =
	      TrackNaiveCollision->Track(particleCloud, handCloud);*/
	      while(pcdInterface->fileIndex < pcdInterface->maxFiles)
		{
		  TrackerManager::GlobalTracker()->GetVisualizer()->spinOnce();
		  PointCloud<PointXYZRGBA>::Ptr kinectCloud =
	      TrackViewer->Track(pcdInterface->GetNextCloud());
		  printf("Spin");
		int i = 0;
		for(i; i < trackerSize; i++)
		  {
		    if(pcdInterface->fileIndex % trackers[i]->loopCounter == 0)
		      {
			printf("Executre");
			if(trackers[i]->InputCloud->points.empty())
			  printf("Shit's empty as fuck dawg");
			else if(trackers[i]->ColliderCloud == NULL)
			  trackers[i]->ThreadedTrack();
			else
			  trackers[i]->ThreadedCollision();
		      }
		  }
		boost::this_thread::sleep(boost::posix_time::milliseconds(milSeconds));
		}
    }

  return 0;
}

