/*
 * Software License Agreement (BSD License)
 *
 */

#ifndef PCL_IMPL_OBJ_TRACKER_H_
#define PCL_IMPL_OBJ_TRACKER_H_

template <typename PointT> bool 
pcl::objTracker<PointT>::addSamples (const PointCloudConstPtr &cloud_in)
{
	bool isSuccess = true;
	if (cloud_in->points.size() < 10)
	{
		isSuccess = false;
	}
	else
	{
		if (obj_model_cloud->points.empty()) //if no model exists, capture the first cloud
		{
			for (size_t i = 0; i < cloud_in->points.size (); i++)
				obj_model_cloud->points.push_back (cloud_in->points[i]);
			pcl::console::print_highlight ("first object frame captured...\n");
		// find features
			normalEstimation(obj_model_cloud, *obj_model_normals);
		} 
		else //if model exists, add new data
		{
			PointCloudPtr new_object_cloud(new PointCloud);
			for (size_t i = 0; i < cloud_in->points.size (); i++)
				new_object_cloud->points.push_back (cloud_in->points[i]);
			// find features
			NormalCloudPtr new_obj_normals(new NormalCloud);
			normalEstimation(new_object_cloud, *new_obj_normals);

			// find transformation
			Eigen::Affine3f new_trans;
			new_trans.Identity();


			// transfrom existing one and combine
			/*for (size_t i = 0; i < cloud_in->points.size (); i++)
			{
				new_object_cloud->points.push_back(pcl::transformPoint(obj_model_cloud->points[i],new_trans));
			}

			obj_model_cloud=new_object_cloud;*/
			// resample


			//
			pcl::console::print_highlight ("new points added to the existing object\n");
		}

		Eigen::Vector4f c;
		pcl::compute3DCentroid<PointT> (*obj_model_cloud, c);
		Eigen::Vector3f oc(c(0),c(1),c(2));
		obj_model_trans = Eigen::Affine3f::Identity ();
		obj_model_trans.translate(oc);


	}

	return(isSuccess);
}

template <typename PointT> bool 
pcl::objTracker<PointT>::initializeCollision (void)
{
	bool isSuccess = true;
	if (obj_model_cloud->points.empty())
	{
		isSuccess = false;
	}
	else
	{
		obj_octree->deleteTree(true);
		obj_octree.reset(new OctreeSearch(32.0f));
		obj_octree->setInputCloud(obj_model_cloud);
		obj_octree->addPointsFromInputCloud ();//this is just for demonstration, actual implementation needs to be done with demeaned cloud and tracking of object
		//setCollisionTH(0.0012);
	}

	return(isSuccess);
}


template <typename PointT> int 
pcl::objTracker<PointT>::collisionDetect (const PointCloudConstPtr &cloud_in, PointCloud &collision_cloud)
{
	int NCollision = 0;
	Eigen::VectorXf collision_check = Eigen::VectorXf::Zero(obj_model_cloud->points.size());
					
	PointCloudPtr trans_input (new PointCloud);
	pcl::transformPointCloud<PointT> (*cloud_in, *trans_input, obj_model_trans.inverse());

	for (size_t i = 1; i < trans_input->points.size (); i++)
	{
		float distance = trans_input->points[i].x*trans_input->points[i].x + trans_input->points[i].y*trans_input->points[i].y
			+ trans_input->points[i].z*trans_input->points[i].z;
		if (distance <0.032)
		{
			int pointIdxNKNSearch;
			float pointNKNSquaredDistance;
			obj_octree->approxNearestSearch (trans_input->points[i], pointIdxNKNSearch, pointNKNSquaredDistance);
			if (pointNKNSquaredDistance<0.0012)
				collision_check(pointIdxNKNSearch)+=1;
		}
	}

	collision_cloud.clear();
	for (size_t i = 1; i < obj_model_cloud->points.size (); i++)
	{
		if (collision_check(i) > 0)
		{
			NCollision++;
			collision_cloud.points.push_back(pcl::transformPoint(obj_model_cloud->points[i],obj_model_trans));
		}
	}

	return (NCollision);
	
}




#define PCL_INSTANTIATE_objTracker(T) template class PCL_EXPORTS pcl::objTracker<T>;
#endif  //#ifndef PCL_IMPL_OBJ_TRACKER_H_