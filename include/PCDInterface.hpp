#ifndef PCD_INTERFACE_H
#define PCD_INTERFACE_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;

class PCDInterface
{
public:
  char rwFunction;
int fileIndex, maxFiles;
  bool Write()
  { return rwFunction == 'w'; }

PCDInterface(char rw, int maxF);
int ShowPCDFiles(int number_of_files);
void RecordPCDFiles(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in);
PointCloud<PointXYZRGBA>::Ptr GetNextCloud();

};
#endif
