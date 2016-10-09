#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>
#include <KDTracker.hpp>
#include <ParticleFilter.hpp>

#include <string>
#include <stdlib.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
  cout << "Eat Your Greens\n";
  
  Tracker* TrackStar = new Tracker("Random Ass Tracker", 'a', 9, 0, 255, 4);
  KDTracker* kdTracker = new KDTracker();
  TrackStar->TrackerAlgorithm = kdTracker;

  Tracker* TrackMarks = new Tracker("Strung out Tracker", 'b', 9, 0, 255, 255);
  std::string file = "data/bbq_pringles.pcd";
  std::string did = "666";
  ParticleFilter* pf = new ParticleFilter(file, did);
  TrackMarks->TrackerAlgorithm = pf;

  //RandomCloud* randomCloud = new RandomCloud();
  //RandomAssTracker->TrackerAlgorithm = randomCloud;

  srand(time(NULL));

//Need a way to register trackers with the TrackerManager so they will respond to//the keyboard callback.
  while(!TrackerManager::GlobalTracker()->GetVisualizer()->wasStopped())
    {
      TrackerManager::GlobalTracker()->VisualizationLoop(true);
      TrackMarks->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
 //TrackStar->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
    }
  return 0;
}

