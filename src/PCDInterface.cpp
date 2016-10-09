#include <PCDInterface.hpp>
#include <TrackerManager.hpp>

using namespace pcl;

PCDInterface::PCDInterface(char rw, int maxF)
{
  rwFunction = rw;
  fileIndex = 0;
  maxFiles = maxF;
  if(Write())
    {
      boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> functionPointerNamedF;
      functionPointerNamedF = boost::bind (&PCDInterface::RecordPCDFiles, this, _1);
      boost::signals2::connection connect = 
    TrackerManager::GlobalTracker()->GetOpenNIGrabber()->registerCallback (functionPointerNamedF);
    }
}

int PCDInterface::ShowPCDFiles(int number_of_files)
{
  /*PointCloud<PointXYZRGBA>::Ptr basic_cloud_ptr(new PointCloud<PointXYZRGBA>);
  for(int i = 0; i < number_of_files+1; ++i)
    {
      std::string filename = "";
      std::stringstream ss;
      ss << i;
    std::string filenum = ss.str();	
    filename = "../data/files/pcd"+filenum+".pcd";
    std::cout<<"Reading file: "<< filename<<std::endl;
    if(io::loadPCDFile<PointXYZRGBA>(filename, *basic_cloud_ptr)==-1)
      {
	std::cout<<"Reading file: "<<filename<<std::endl;
	return(-1);
      }
    viewer.showCloud(basic_cloud_ptr);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }
  while(!viewer.wasStopped())
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      }*/
}

void PCDInterface::RecordPCDFiles(const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  if(TrackerManager::GlobalTracker()->GetOpenNIGrabber()->isRunning())
    {
      std::stringstream ss;
      ss << fileIndex;
      PointCloud<PointXYZRGBA>::Ptr cloud_filtered (new PointCloud<PointXYZRGBA>);
      pcl::PassThrough<pcl::PointXYZRGBA> pass;
      pass.setInputCloud (cloud_in);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, TrackerManager::GlobalTracker()->GetZDepth());
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
      io::savePCDFileBinary ("../data/files/pcd"+ss.str()+".pcd", *cloud_filtered);
      fileIndex++;
      if(fileIndex>maxFiles)
	{
	  TrackerManager::GlobalTracker()->GetOpenNIGrabber()-> stop();
	}
      /*else
	{
	  boost::this_thread::sleep (boost::posix_time::milliseconds (5));
	  std::cout<<"Recording in 3 ..."<<std::endl;	
	  boost::this_thread::sleep (boost::posix_time::milliseconds (5));
	  std::cout<<"Recording in 2 ..."<<std::endl;	
	  boost::this_thread::sleep (boost::posix_time::milliseconds (5));
	  std::cout<<"Recording in 1 ..."<<std::endl;	
	  }*/
    }
}

PointCloud<PointXYZRGBA>::Ptr PCDInterface::GetNextCloud()
{
  PointCloud<PointXYZRGBA>::Ptr basic_cloud_ptr(new PointCloud<PointXYZRGBA>);
  std::stringstream ss;
  ss << fileIndex;
  std::string filename = "../data/files/pcd"+ss.str()+".pcd";
  fileIndex++;
   if(io::loadPCDFile<PointXYZRGBA>(filename, *basic_cloud_ptr)==-1)
      {
	/*std::cout<<"End of dataset. Loopin'..."<<filename<<std::endl;
	fileIndex = 0;
	ss.clear();
	ss << fileIndex;
	filename = "../data/files/pcd"+ss.str()+".pcd";
	io::loadPCDFile<PointXYZRGBA>(filename, *basic_cloud_ptr);*/
	std::cout << "staphit" << std::endl;
	//TrackerManager::GlobalTracker()->GetOpenNIGrabber()->stop();
      }
   return basic_cloud_ptr; 
}
