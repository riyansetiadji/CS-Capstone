#include <Container.hpp>

using namespace pcl;

Container::Container()
{}

Container::Container(std::string trackerName, char callback, 
		 float r, float g, float b, int ps, bool en, float h)
  {
    callbackKey = callback;
    visualizerTextHeight = h;

    name = trackerName;
    red = r; green = g; blue = b;
    pointSize = ps;
    enabled = en;
  }
