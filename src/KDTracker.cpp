#include <KDTracker.hpp>
#include <pcl/filters/extract_indices.h>

using namespace pcl;
using namespace std;
KDTracker::KDTracker(std::string handPath, std::string objectPath)
{
  hand_h= 0.0;
  hand_s = 0.0;
  hand_v = 0.0;
  object_h = 0.0;
  object_s = 0.0;
  object_v = 0.0;
  distance_threshold = 12;
  point_color_threshold = 12;
  region_color_threshold = 12;
  min_cluster_size = 100;
  max_cluster_size = 50000;
  threshold = 2.7;

  hand_cloud.reset(new PointCloud<PointXYZRGBA>);
  hand_object_cloud.reset(new PointCloud<PointXYZRGBA>);
  
  io::loadPCDFile<PointXYZRGBA>(handPath, *hand_cloud);
  io::loadPCDFile<PointXYZRGBA>(objectPath, *hand_object_cloud);  

  // last parameter, true to show resulting cloud. false to not
  getAvgHSV(hand_cloud,&hand_h,&hand_s,&hand_v,0);
  std::cout<<"hand hue:"<<hand_h<<std::endl;
  std::cout<<"hand saturation:"<<hand_s<<std::endl;
  std::cout<<"hand value:"<<hand_v<<std::endl;

  // last parameter, true to show resulting cloud. false to not
  getObjectAvgHSV(hand_object_cloud,hand_h,&object_h,&object_s,&object_v,1);
  std::cout<<"object hue:"<<object_h<<std::endl;
  std::cout<<"object saturation:"<<object_s<<std::endl;
  std::cout<<"object value:"<<object_v<<std::endl;

  //tree = boost::shared_ptr<search::Search<PointXYZRGBA> > (new search::KdTree<PointXYZRGBA>);
}

PointCloud<PointXYZRGBA>::Ptr
KDTracker::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  //ReturnCloud = filter_table(cloud_in);
  ReturnCloud = filterSetOfPcdWithClusters(cloud_in,hand_h,object_h,0);
  //ReturnCloud = filterSetOfPcdWithPoints(cloud_in,hand_h,object_h,0);
  return ReturnCloud;
}
PointCloud<PointXYZRGBA>::Ptr KDTracker::filter_table( const PointCloud<PointXYZRGBA>::ConstPtr &cloud_in)
{
  PointCloud<PointXYZRGBA>::Ptr filtered_cloud (new PointCloud<PointXYZRGBA>);
 PointCloud<PointXYZRGBA>::Ptr tmp_cloud (new PointCloud<PointXYZRGBA>);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_in);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    //return (-1);
  }

  

 
 

  pcl::ExtractIndices<PointXYZRGBA> eifilter (true); // Initializing with true will allow us to extract the removed indices
  eifilter.setInputCloud (cloud_in);
  eifilter.setIndices (inliers);
  eifilter.setNegative(true);
  eifilter.filter (*tmp_cloud);
  // The resulting cloud_out contains all points of cloud_in that are indexed by indices_in
 
  // The indices_rem array indexes all points of cloud_in that are not indexed by indices_in
  
  // for (size_t i = 0; i < indices_rem.size (); ++i)
     //filtered_cloud->points.push_back(cloud_in->points[indices_rem[i]]);
return tmp_cloud;

}
void KDTracker::RGBToHSV(float r, float g, float b, float *h, float *s, float *v)
{
  float max = r;
  if (max < g) max = g;
  if (max < b) max = b;
  float min = r;
  if (min > g) min = g;
  if (min > b) min = b;

    

  *h = 0;
  if (max == min) h = 0;
  else if (max == r) {
    *h = 60 * (g - b)/(max - min);
    if (*h < 0) *h += 360;
    if (*h >= 360) *h -= 360;
  } else if (max == g) {
    *h = 60 * (b - r) / (max - min) + 120;
  } else if (max == b) {
    *h = 60 * (r - g) / (max - min) + 240;
  }

  if (max == 0) *s = 0;
  else *s = (1 - (min / max));

  *v = max/255;
}

void KDTracker::getAvgHSV(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double *h_avg, double *s_avg, double *v_avg,int showcld)
{
  *h_avg=0.0;
  *s_avg=0.0;
  *v_avg=0.0;
  double counter = 0;
  float r;
  float g;
  float b;
  float h;
  float s;
  float v;
  for(int i =0; i < cloud->points.size(); i=i+1)
    {
 
      r =(float) cloud->points[i].r;
      g = (float) cloud->points[i].g;
      b = (float) cloud->points[i].b;
      // std::cout<<r<<","<<g<<","<<b<<";";
      if(r> 0 && r <= 255 && g> 0 && g <= 255 && b> 0 && b <= 255)
	{
	  counter++;
	  RGBToHSV(r,g,b,&h,&s,&v);
	  *h_avg += (double)h;
	  *s_avg += (double)s;
	  *v_avg += (double)v;
                            
    
	}
    }
  *h_avg/=counter;
  *s_avg/=counter;
  *v_avg/=counter;
  if(showcld)
    {
      
    }
                    
  /*
    std::cout<<"---------hsv average--------"<<std::endl;
    std::cout<<"hue average: "<<*h_avg<<std::endl;
    std::cout<<"saturation average: "<<*s_avg<<std::endl;
    std::cout<<"value avaerage: "<<*v_avg<<std::endl;*/
}

void KDTracker::getAvgHSVForOnePoints(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double *h_avg, double *s_avg, double *v_avg,int showcld)
{  
  *h_avg=0.0;
  *s_avg=0.0;
  *v_avg=0.0;
  double counter = 0;
  float r;
  float g;
  float b;
  float h;
  float s;
  float v;
  for(int i =0; i < cloud->points.size(); i=i+4)
    {
 
      r =(float) cloud->points[i].r;
      g = (float) cloud->points[i].g;
      b = (float) cloud->points[i].b;
      // std::cout<<r<<","<<g<<","<<b<<";";
      if(r> 0 && r <= 255 && g> 0 && g <= 255 && b> 0 && b <= 255)
	{
	  counter++;
	  RGBToHSV(r,g,b,&h,&s,&v);
	  *h_avg += (double)h;
	  *s_avg += (double)s;
	  *v_avg += (double)v;
                            
    
	}
      else
	{
	  return;
	}
    }
  *h_avg/=counter;
  *s_avg/=counter;
  *v_avg/=counter;
  if(showcld)
    {
    }
                    
  /*
    std::cout<<"---------hsv average--------"<<std::endl;
    std::cout<<"hue average: "<<*h_avg<<std::endl;
    std::cout<<"saturation average: "<<*s_avg<<std::endl;
    std::cout<<"value avaerage: "<<*v_avg<<std::endl;*/
}

void KDTracker::getObjectAvgHSV(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,double hue,double *objectH, double *objectS,double *objectV,int showcld)
{
  //a percentage from 0 to 1. how close to scanned average does the point need to 
  //used for region segmentation
  search::Search <PointXYZRGBA>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGBA> > (new search::KdTree<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr tmp_cloud(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr filtered_cloud(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr segmented_cloud(new PointCloud<PointXYZRGBA>);

  //depth filter.
  IndicesPtr indices (new std::vector <int>);
  PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  //region segmentation
  RegionGrowingRGB<PointXYZRGBA> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distance_threshold);
  reg.setPointColorThreshold (point_color_threshold);
  reg.setRegionColorThreshold (region_color_threshold);
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);

  std::vector <PointIndices> clusters;
  reg.extract (clusters);

  //holds the cloud that color codes the different clusters. replace finalcloud with this if you want to see clusters
            
  segmented_cloud = reg.getColoredCloudRGBA ();
            
  int clusterIncluded = 0;

  for(int i =0; i < clusters.size(); i++)
    {
      int counter = 9999;

      for(int j=0; j< clusters[i].indices.size(); j++)
	{

	  tmp_cloud->points.push_back(cloud->points[clusters[i].indices[j]]);
                    	

	}
      /*
	if(showcld)
	{
	while(counter > 0 )
	{
	viewer.showCloud (tmp_cloud);
	counter--;
	//std::cout<<counter<<std::endl;
	}
	}*/

      double h_average = 0.0;
      double s_average = 0.0;
      double v_average = 0.0;
      getAvgHSV(tmp_cloud,&h_average,&s_average,&v_average,0);
      //std::cout<<h_average<<", ";
      if(h_average > (hue+(hue*threshold))  || h_average < (hue-(hue*threshold)) )
	{
	  clusterIncluded++;
	  *objectH += h_average;
	  *objectS += s_average;
	  *objectV += v_average;
	  for(int j=0; j< clusters[i].indices.size(); j++)
	    {
	      filtered_cloud->points.push_back(cloud->points[clusters[i].indices[j]]);
	    }
	}
      tmp_cloud->points.clear();
    }

  *objectH/=clusterIncluded;
  *objectS/=clusterIncluded;
  *objectV/=clusterIncluded;
  if(showcld)
    {
    }
}

PointCloud<PointXYZRGBA>::Ptr KDTracker::PointFilter(const PointCloud<PointXYZRGBA>::Ptr &cloud,double handH, double objectH)
{
  PointCloud<PointXYZRGBA>::Ptr filtered_cloud(new PointCloud<PointXYZRGBA>);

   pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
   kdtree.setInputCloud (cloud);

  for(int i =0; i < cloud->points.size();i++)
  {

    float radius = .01f;

    std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

    if( kdtree.radiusSearch (cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 55)
    {
      //std::cout<<"hereeeeeee"<<std::endl;
      filtered_cloud->points.push_back(cloud->points[i]); 
    }
  }
  return filtered_cloud;
}


PointCloud<PointXYZRGBA>::Ptr KDTracker::filterSetOfPcdWithClusters(const PointCloud<PointXYZRGBA>::Ptr &cloud, double handH,double objectH,int showcld)
{
  //finished loading cloud
  //a percentage from 0 to 1. how close to scanned average does the point need to be          
  //used for region segmentation
  search::Search <PointXYZRGBA>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGBA> > (new search::KdTree<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr tmp_cloud(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr filtered_cloud(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr segmented_cloud(new PointCloud<PointXYZRGBA>);
  hand_subtraction.reset(new PointCloud<PointXYZRGBA>);

  //depth filter.
  IndicesPtr indices (new std::vector <int>);
  PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  //region segmentation
  RegionGrowingRGB<PointXYZRGBA> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distance_threshold);
  reg.setPointColorThreshold (point_color_threshold);
  reg.setRegionColorThreshold (region_color_threshold);
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);
           
  std::vector <PointIndices> clusters;
  reg.extract (clusters);

  //holds the cloud that color codes the different clusters. replace finalcloud with this if you want to see clusters
            
  segmented_cloud = reg.getColoredCloudRGBA ();
            
  int clusterIncluded = 0;
  std::cout<<"Cluster Size: "<<clusters.size()<<std::endl;
  for(int i =0; i < clusters.size(); i++)
    {

      for(int j=0; j< clusters[i].indices.size(); j++)
	{

	  tmp_cloud->points.push_back(cloud->points[clusters[i].indices[j]]);
                    	
	}
                    
      double h_average = 0.0;
      double s_average = 0.0;
      double v_average = 0.0;
      getAvgHSV(tmp_cloud,&h_average,&s_average,&v_average,0);
                    
      double distanceToHand = 0.0;
      double distanceToObject = 0.0;

      distanceToHand = std::abs(handH - h_average);
      distanceToObject = std::abs(objectH - h_average);

      if(distanceToHand < distanceToObject)
	{
	  for(int j=0; j< clusters[i].indices.size(); j++)//hand cloud
	    {
	      cloud->points[clusters[i].indices[j]].r = 255;
	      cloud->points[clusters[i].indices[j]].g = 0;
	      cloud->points[clusters[i].indices[j]].b = 0;
                    		
	      filtered_cloud->points.push_back(cloud->points[clusters[i].indices[j]]);
       
	    }
      
	}
      else
	{
	  for(int j=0; j< clusters[i].indices.size(); j++)//object cloud
	    {
	      cloud->points[clusters[i].indices[j]].r = 0;
	      cloud->points[clusters[i].indices[j]].g = 255;
	      cloud->points[clusters[i].indices[j]].b = 0;
                    		
	      hand_subtraction->points.push_back(cloud->points[clusters[i].indices[j]]);
	    }

	}
      tmp_cloud->points.clear();
    }
 std::cout<<"hand cloud point size: "<<filtered_cloud->points.size()<<std::endl;
 std::cout<<"object cloud point size: "<<hand_subtraction->points.size()<<std::endl;
  return filtered_cloud;
}

PointCloud<PointXYZRGBA>::Ptr KDTracker::filterSetOfPcdWithPoints(const PointCloud<PointXYZRGBA>::Ptr &cloud,double handH,double objectH,int showcld)
{
  PointCloud<PointXYZRGBA>::Ptr filtered_cloud(new PointCloud<PointXYZRGBA>);
  //std::cout << "In the function" << std::endl;
  //a percentage from 0 to 1. how close to scanned average does the point need to be 
  //used for region segmentation
  PointCloud<PointXYZRGBA>::Ptr tmp_cloud(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr segmented_cloud(new PointCloud<PointXYZRGBA>);
  hand_subtraction.reset(new PointCloud<PointXYZRGBA>);

  for(int i=0; i < cloud->points.size(); i++)
    {

      tmp_cloud->points.push_back(cloud->points[i]);
                    	
      double h_average = 0.0;
      double s_average = 0.0;
      double v_average = 0.0;
      getAvgHSVForOnePoints(tmp_cloud,&h_average,&s_average,&v_average,0);
	                    
      double distanceToHand = 0.0;
      double distanceToObject = 0.0;

      distanceToHand = std::abs(handH - h_average);
      distanceToObject = std::abs(objectH - h_average);
      float omitThreshold = .2;
      if(distanceToHand < distanceToObject)
    	{
                cloud->points[i].r = 0;
                cloud->points[i].g = 255;
                cloud->points[i].b = 0;
                                
                filtered_cloud->points.push_back(cloud->points[i]); 
    	}
      else
	{              
	      hand_subtraction->points.push_back(cloud->points[i]);		
	}
      tmp_cloud->points.clear();

    }
  //std::cout << "Made it" << std::endl;                           
  //filtered_cloud->points.clear();
  return PointFilter(filtered_cloud,handH,objectH);
  //return filtered_cloud;
}

PointCloud<PointXYZRGBA>::Ptr KDTracker::getObjectCloud()
{
  return hand_subtraction;
}


Eigen::Affine3f KDTracker::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void KDTracker::PrintAlgorithm()
{
  
}
