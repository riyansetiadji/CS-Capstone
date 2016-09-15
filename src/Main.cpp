#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>
#include <RandomCloud.hpp>

#include <string>
#include <stdlib.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
  cout << "Eat Your Greens\n";
  
  Tracker* RandomAssTracker = new Tracker("Random Ass Tracker", 'a', 9, 0, 255, 4);
  RandomCloud* randomCloud = new RandomCloud();
  RandomAssTracker->TrackerAlgorithm = randomCloud;

  Tracker* AnotherRandomAssTracker = 
    new Tracker("Another Random Ass Tracker", 'b', 128, 128, 0, 4);
  RandomCloud* AnotherRandomCloud = new RandomCloud();
  AnotherRandomAssTracker->TrackerAlgorithm = AnotherRandomCloud;

  Tracker* YetAnotherRandomAssTracker = 
    new Tracker("Yet Another Random Ass Tracker", 'c', 255, 255, 255, 4);
  RandomCloud* YetAnotherRandomCloud = new RandomCloud();
  YetAnotherRandomAssTracker->TrackerAlgorithm = YetAnotherRandomCloud;

  srand(time(NULL));

  //Need a way to register trackers with the TrackerManager so they will respond to 
  //the keyboard callback.
  while(!TrackerManager::GlobalTracker()->GetVisualizer()->wasStopped())
    {
      TrackerManager::GlobalTracker()->VisualizationLoop();
        RandomAssTracker->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	AnotherRandomAssTracker->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
	YetAnotherRandomAssTracker->Track(TrackerManager::GlobalTracker()->GetKinectCloud());
    }
  return 0;
}
