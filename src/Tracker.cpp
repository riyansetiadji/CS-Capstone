#include <Tracker.hpp>
#include <string>

using namespace pcl;

Tracker::Tracker()
  {
    Target = TARGET_UNKNOWN;
    TargetTransform = Eigen::Affine3f::Identity ();
  }

Tracker::Tracker(std::string trackerName, std::string shapeName, 
		 float r, float g, float b, int ps)
  {
    Target = TARGET_UNKNOWN;
    TargetTransform = Eigen::Affine3f::Identity ();

    name = trackerName;
    shape = shapeName;
    red = r; green = g; blue = b;
    pointSize = ps;
  }

void Tracker::Track(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  //Will also need to do some control flow based on enumeration state of Target
  //If this depends on the nature of the algorithm we may be in trouble
  if(Target == TARGET_UNKNOWN)
    {
      TargetCloud = TrackerAlgorithm->Execute(cloud_in);
      TrackerAlgorithm->ComputeTransform();
      Target = TARGET_TRACKING;
    }
  else if(Target == TARGET_IDENTIFYING)
    {
      //TrackerAlgorithm->ComputeTransform();
    }
  else
    {
      //TrackerAlgorithm->ComputeTransform();
    }
  //TrackerAlgorithm->Execute(cloud_in);
  //Need to have a way to reconcile the cloud_in with the captured target
  //in order to produce the new output, something like
  //OutputCloud = cloud_in - TargetCloud;
}

void Tracker::KeyboardCallback ()
{

}

/*void Tracker::cloud_callback(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{ 
  
  if (performSegmentation)
    {
      tableSegmenter->Execute(cloud_in);
      performSegmentation = false;
      return;
    }
  			
			// filter against table plane
  double sqr_abc = 1/sqrt(table_plane_coef->values[0] * 
			  table_plane_coef->values[0] + 
			  table_plane_coef->values[1] * 
			  table_plane_coef->values[1] +
			  table_plane_coef->values[2] * 
			  table_plane_coef->values[2]);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
	if (filter_mask[i])
	  {
        // examine point with respect to plane
	    double dummy = (table_plane_coef->values[0] * 
			    cloud_in->points[i].x + 
			    table_plane_coef->values[1] * 
			    cloud_in->points[i].y + 
			    table_plane_coef->values[2] * 
			    cloud_in->points[i].z + 
			    table_plane_coef->values[3]) * sqr_abc;
	    if ( dummy < 0.015)
	      filter_mask[i] = false;
	  }
    }

  // copy cloud for skin color filter
  hand1Cloud.reset(new PointCloud<PointXYZRGBA>);
  hand2Cloud.reset(new PointCloud<PointXYZRGBA>);
  filteredCloud.reset(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr temp_cloud (new PointCloud<PointXYZRGBA>);
  temp_cloud->height = CLOUD_HEIGHT;
  temp_cloud->width = CLOUD_WIDTH;
  
  PointXYZRGBA dummyPt;
  dummyPt.x = dummyPt.y = dummyPt.z = 0;
  dummyPt.r = dummyPt.g = dummyPt.b = 0;

  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      if (filter_mask[i])
	{
	  temp_cloud->points.push_back(cloud_in->points[i]);
	  filteredCloud->points.push_back(cloud_in->points[i]);
	}
      else
	temp_cloud->points.push_back(dummyPt);
    }

  if (Hand == HAND_TRACKING)
    {
      //handAlgorithm->Execute(temp_cloud);
    }
  else if(Hand != HAND_TRACKING) // if not tracking, use start position
    {
      Eigen::Vector3f hand_c(hand_trans.translation());  
      hand1Cloud.reset(new PointCloud<PointXYZRGBA>);
      for (size_t i = 0; i < filteredCloud->points.size (); ++i)
	{
	  double distance = (filteredCloud->points[i].x-hand_c(0)) * 
	    (filteredCloud->points[i].x-hand_c(0)) + 
	    (filteredCloud->points[i].y-hand_c(1)) * 
	    (filteredCloud->points[i].y-hand_c(1)) + 
	    (filteredCloud->points[i].z-hand_c(2)) * 
	    (filteredCloud->points[i].z-hand_c(2));
	  
	  if (distance < 0.0256) 
	    hand1Cloud->points.push_back(filteredCloud->points[i]);
	}
    }			

  if (Hand == HAND_IDENTIFYING)
    {
      if (skin_Filter.addSamples(hand_cloud) )
	{
	  cout<< "total number of samples:  " << skin_Filter.getSampleNumbers() << endl;
	  Hand = HAND_UNKNOWN;
	}
      else
	{
	  pcl::console::print_highlight ("sample pool full\n start tracking\n");
	  Hand = HAND_TRACKING;
	}
    }

  // object scanning 
  if (Object == OBJ_SCANNING)
    {	
      // cluster
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<RefPointType> ec;
      pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType>);
      ec.setClusterTolerance (0.02); // 5cm
      ec.setMinClusterSize (50);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (filteredCloud);
      ec.extract (cluster_indices);

      Eigen::Vector4f c;
      Eigen::Vector4f c_new;
      Eigen::Vector3f tc(table_trans.translation()); //for object scan
      //Eigen::Vector3f tc(0.0,0.0,0.0);//select the cloud closest to the camera
      pcl::compute3DCentroid<RefPointType> (*filteredCloud, cluster_indices[0].indices, c);
      int segment_index = 0;
      double segment_distance = (c(0)-tc(0)) * (c(0)-tc(0)) + (c(1)-tc(1)) * (c(1)-tc(1)) + (c(1)-tc(1)) * (c(1)-tc(1));
      for (size_t i = 1; i < cluster_indices.size (); i++)
	{
	  pcl::compute3DCentroid<RefPointType> (*filteredCloud, cluster_indices[i].indices, c);
	  double distance = (c(0)-tc(0)) * (c(0)-tc(0)) + (c(1)-tc(1)) * (c(1)-tc(1)) + (c(1)-tc(1)) * (c(1)-tc(1));
	  if (distance < segment_distance)
	    {
	      segment_index = int (i);
	      segment_distance = distance;
	    }
	}

      obj_model_cloud.reset(new PointCloud<PointXYZRGBA>);
      for (size_t i = 0; i < cluster_indices[segment_index].indices.size (); i++)
	obj_model_cloud->points.push_back (filteredCloud->points[cluster_indices[segment_index].indices[i]]);

      if (obj_Tracker.addSamples(obj_model_cloud))
	{
	  obj_model_cloud = obj_Tracker.getObjCloud();
	}
      else
	{
	  obj_model_cloud.reset(new PointCloud<PointXYZRGBA>);
	  PCL_WARN ("no object found");
	}

      //Object = OBJ_UNKNOWN;
      if (obj_Tracker.initializeTracking())
	{
	  pcl::console::print_highlight ("start object Tracking ... \n");
	  Object = OBJ_TRACKING;
	}
      else
	Object = OBJ_UNKNOWN;

    } // end of object scanning code

  if (Object == OBJ_TRACKING)
    {
      obj_Tracker.setInputCloud(filteredCloud);
      obj_Tracker.updatePoseFromInputCloud();
      obj_model_cloud = obj_Tracker.getObjCloud();
    }
				

  // do some collision detection here
  if ( (Hand == HAND_TRACKING) && tracking_interaction)
    {
      obj_Tracker.collisionDetect(hand_cloud,*obj_affordance);
    }

  //objAlgorithm->Execute();
  //handAlgorithm->Execute();
  //collisionDetect->Execute();
}
*/
