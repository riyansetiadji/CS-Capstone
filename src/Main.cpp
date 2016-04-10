#include <TrackerManager.hpp>
#include <Tracker.hpp>
#include <TableSegmenter.hpp>

#include <string>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
  cout << "Eat Your Greens\n";

  TrackerManager *MainTracker = new TrackerManager();
  Tracker *TableTracker = new Tracker("TableSegmenter", "A", 255, 0, 128, 2);
  TableSegmenter *tableSeg = new TableSegmenter();
  TableTracker->TrackerAlgorithm = tableSeg;

  MainTracker->NextTracker = TableTracker;

  while (!MainTracker->visualizer->wasStopped ())
    {
      MainTracker->VisualizationLoop();
    }

  return 0;
}
