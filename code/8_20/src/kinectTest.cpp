#include <iostream>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>
#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/pcl_search.h>

#include <pcl/apps/hand_obj_tracker/skinFilter.h>
#include <pcl/apps/hand_obj_tracker/objTracker.h>

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)


//class newVisualizerKeyBinding : public pcl::visualization::PCLVisualizerInteractorStyle
//{
//
//}
#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

template <typename PointType>
class KinectTracking
{
  enum objState
  {
    OBJ_UNKNOWN,
		OBJ_IDENTIFIED,
    OBJ_TRACKING,
		OBJ_SCANNING,
  };

  enum handState
  {
    HAND_UNKNOWN,
    HAND_IDENTIFYING,
    HAND_TRACKING,
  };

  public:
		typedef pcl::PointXYZRGBA RefPointType;
		typedef pcl::PointCloud<PointType> Cloud;
		typedef pcl::PointCloud<RefPointType> RefCloud;
		typedef typename RefCloud::Ptr RefCloudPtr;
		typedef typename RefCloud::ConstPtr RefCloudConstPtr;
		typedef typename Cloud::Ptr CloudPtr;
		typedef typename Cloud::ConstPtr CloudConstPtr;

    KinectTracking(int argc, char** argv)
		: z_limit_ (2.0)
		,	bg_initialized (false)
		,	displayBG (true)
		, tracking_interaction (false)
		{
			main_viz.reset (new pcl::visualization::PCLVisualizer (argc, argv, "Tracking Viewer"));
			// initialize background
			table_trans = Eigen::Affine3f::Identity ();
			

			// initialize object
			object_status = OBJ_UNKNOWN;
			obj_model_cloud.reset(new RefCloud); // here to insert object loading code
			obj_affordance.reset(new RefCloud);
			
			hand_status = HAND_UNKNOWN;
			hand_cloud.reset(new RefCloud);
			hand_cloud2.reset(new RefCloud);
			hand_trans = Eigen::Affine3f::Identity ();


		}


		// grid down sampling
		void 
		gridSampleApprox (const CloudConstPtr &cloud_in, Cloud &result, double leaf_size = 0.01)
		{
			pcl::ApproximateVoxelGrid<RefPointType> grid;
			grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
			grid.setInputCloud (cloud_in);
			grid.filter (result);
		}


		void 
		segment_table_plane(const RefCloudConstPtr & cloud_in)
		{
			pcl::console::print_highlight ("segmenting plane...\n");

			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			table_plane_coef.reset(new pcl::ModelCoefficients ());
				// Create the segmentation object
			pcl::SACSegmentation<RefPointType> seg;
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (0.002);
			seg.setInputCloud (cloud_in);
			seg.segment (*inliers, *table_plane_coef);


			RefCloudPtr table_cloud(new RefCloud);
			RefCloudPtr proj_cloud(new RefCloud);
			pcl::ExtractIndices<RefPointType> extract;
			extract.setInputCloud (cloud_in);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*table_cloud);

			if (table_plane_coef->values[3]<0) // make sure the camera is always on the positive side of the reference plane
			{
				table_plane_coef->values[0] = -1.0*table_plane_coef->values[0]; table_plane_coef->values[1] = -1.0*table_plane_coef->values[1];
				table_plane_coef->values[2] = -1.0*table_plane_coef->values[2]; table_plane_coef->values[3] = -1.0*table_plane_coef->values[3];
			}

			pcl::ProjectInliers<RefPointType> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (table_cloud);
			proj.setModelCoefficients (table_plane_coef);
			proj.filter (*proj_cloud);
		
			Eigen::Vector4f table_c;
			pcl::compute3DCentroid(*proj_cloud,table_c);
			Eigen::Vector3f origin(table_c(0),table_c(1),table_c(2));
			table_trans.translate(origin);
			Eigen::Vector3f zdir(table_plane_coef->values[0],table_plane_coef->values[1],table_plane_coef->values[2]);
			zdir.normalize();
			Eigen::Vector4f max_pt;
			pcl::getMaxDistance (*proj_cloud, table_c, max_pt);
			Eigen::Vector3f ydir(max_pt(0)-table_c(0),max_pt(1)-table_c(1),max_pt(2)-table_c(2));
			ydir.normalize();
			Eigen::Affine3f trans;
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(ydir, zdir, origin, trans);
			table_trans = trans.inverse();

			Eigen::Vector3f handStart(-0.1, -0.2, 0.75);//0.9
			hand_trans.translate(handStart);
		}

		void 
		cloud_cb (const RefCloudConstPtr & cloud_in)
		{
			boost::mutex::scoped_lock lock (cld_mutex);
			double start = pcl::getTime ();
			FPS_CALC ("cloud callback");

			RefCloudPtr dist_filtered_cloud (new RefCloud);
			bool filter_mask[CLOUD_WIDTH * CLOUD_HEIGHT];
			// cloud_in height = 240, width = 320
			for (size_t i = 0; i < cloud_in->points.size (); ++i)
			{
				if ( cloud_in->points[i].z < z_limit_)
				{
					dist_filtered_cloud->points.push_back(cloud_in->points[i]);
					filter_mask[i] = true;
				}
				else
					filter_mask[i] = false;
			}

			// setup table plane filter
			if (!bg_initialized)
			{
				segment_table_plane(dist_filtered_cloud);
				bg_initialized = true;
				return;
			}

			g_cloud.reset(new RefCloud);
			if (displayBG)
				g_cloud.swap(dist_filtered_cloud);

			
			// filter against table plane
			double sqr_abc = 1/sqrt(table_plane_coef->values[0] * table_plane_coef->values[0] + table_plane_coef->values[1] * table_plane_coef->values[1]
														+table_plane_coef->values[2] * table_plane_coef->values[2]);
			for (size_t i = 0; i < cloud_in->points.size (); ++i)
			{
				if (filter_mask[i])
				{
					// examine point with respect to plane
					double dummy = (table_plane_coef->values[0] * cloud_in->points[i].x + table_plane_coef->values[1] * cloud_in->points[i].y
											+ table_plane_coef->values[2] * cloud_in->points[i].z + table_plane_coef->values[3]) * sqr_abc;
					if ( dummy < 0.15)
						filter_mask[i] = false;
				}
			}

			// copy cloud for skin color filter
			hand_cloud.reset(new RefCloud);
			hand_cloud2.reset(new RefCloud);
			filtered_cloud.reset(new RefCloud);
			RefCloudPtr temp_cloud (new RefCloud);
			temp_cloud->height = CLOUD_HEIGHT;
			temp_cloud->width = CLOUD_WIDTH;
			RefPointType dummyPt;
			dummyPt.x = dummyPt.y = dummyPt.z = 0;
			dummyPt.r = dummyPt.g = dummyPt.b = 0;
			for (size_t i = 0; i < cloud_in->points.size (); ++i)
			{
				if (filter_mask[i])
				{
					temp_cloud->points.push_back(cloud_in->points[i]);
					filtered_cloud->points.push_back(cloud_in->points[i]);
				}
				else
					temp_cloud->points.push_back(dummyPt);
			}

			if (hand_status == HAND_TRACKING)
			{
				skin_Filter.setInputCloud(temp_cloud);	
				skin_Filter.identifySkin(*hand_cloud, *hand_cloud2, *filtered_cloud);

			}
			else if(hand_status!=HAND_TRACKING) // if not tracking, use start position
			{
				Eigen::Vector3f hand_c(hand_trans.translation());  
				hand_cloud.reset(new RefCloud);
				for (size_t i = 0; i < filtered_cloud->points.size (); ++i)
				{
					double distance = (filtered_cloud->points[i].x-hand_c(0)) * (filtered_cloud->points[i].x-hand_c(0)) + 
						(filtered_cloud->points[i].y-hand_c(1)) * (filtered_cloud->points[i].y-hand_c(1)) + 
						(filtered_cloud->points[i].z-hand_c(2)) * (filtered_cloud->points[i].z-hand_c(2));
					if (distance < 0.0256) 
						hand_cloud->points.push_back(filtered_cloud->points[i]);
				}
			}			

			if (hand_status == HAND_IDENTIFYING)
			{
				if ( skin_Filter.addSamples(hand_cloud) )
				{
					cout<< "totoal number of samples:  " << skin_Filter.getSampleNumbers() << endl;
					hand_status = HAND_UNKNOWN;
				}
				else
				{
					pcl::console::print_highlight ("sample pool full\n start tracking\n");
					hand_status = HAND_TRACKING;
				}
			}

			// object scanning 
			if (object_status == OBJ_SCANNING)
			{	
				// cluster
				std::vector<pcl::PointIndices> cluster_indices;
				pcl::EuclideanClusterExtraction<RefPointType> ec;
				pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType>);
    		ec.setClusterTolerance (0.02); // 5cm
				ec.setMinClusterSize (50);
				ec.setMaxClusterSize (25000);
				ec.setSearchMethod (tree);
				ec.setInputCloud (filtered_cloud);
				ec.extract (cluster_indices);

				Eigen::Vector4f c;
				Eigen::Vector4f c_new;
				Eigen::Vector3f tc(table_trans.translation()); //for object scan
				//Eigen::Vector3f tc(0.0,0.0,0.0);//select the cloud closest to the camera
				pcl::compute3DCentroid<RefPointType> (*filtered_cloud, cluster_indices[0].indices, c);
				int segment_index = 0;
				double segment_distance = (c(0)-tc(0)) * (c(0)-tc(0)) + (c(1)-tc(1)) * (c(1)-tc(1)) + (c(1)-tc(1)) * (c(1)-tc(1));
				for (size_t i = 1; i < cluster_indices.size (); i++)
				{
					pcl::compute3DCentroid<RefPointType> (*filtered_cloud, cluster_indices[i].indices, c);
					double distance = (c(0)-tc(0)) * (c(0)-tc(0)) + (c(1)-tc(1)) * (c(1)-tc(1)) + (c(1)-tc(1)) * (c(1)-tc(1));
					if (distance < segment_distance)
					{
						segment_index = int (i);
						segment_distance = distance;
					}
				}

				obj_model_cloud.reset(new RefCloud);
				for (size_t i = 0; i < cluster_indices[segment_index].indices.size (); i++)
						obj_model_cloud->points.push_back (filtered_cloud->points[cluster_indices[segment_index].indices[i]]);

				if (obj_Tracker.addSamples(obj_model_cloud))
				{
					obj_model_cloud = obj_Tracker.getObjCloud();
				}
				else
				{
					obj_model_cloud.reset(new RefCloud);
					PCL_WARN ("no object found");
				}

				object_status = OBJ_UNKNOWN;
			} // end of object scanning code
				

			// do some collision detection here
			if ( (hand_status == HAND_TRACKING) && tracking_interaction)
			{
				obj_Tracker.collisionDetect(hand_cloud,*obj_affordance);
			}

				
			// get computation time
			double end = pcl::getTime ();
			computation_time_ = (end - start)*1000;
		}

		//Simple callbacks.
		void 
		keyboard_callback (const pcl::visualization::KeyboardEvent& event)
		{

			if (event.keyUp())
			{
				switch (event.getKeyCode())
				{
					case 109: //"m"
						if (object_status == OBJ_UNKNOWN)
						{
							pcl::console::print_highlight ("scanning object ... ");
							object_status = OBJ_SCANNING;
						}
						break;
					case 105: //"i"
						if (tracking_interaction)
						{
							tracking_interaction = false;
							obj_affordance.reset(new RefCloud);
							pcl::console::print_highlight ("stop tracking hand object interaction \n");
						}
						else
						{
							if (obj_Tracker.initializeCollision())
							{
								tracking_interaction = true;
								pcl::console::print_highlight ("start tracking hand object interaction \n");
							}

						}
						break;
					case 112: //"p"
						bg_initialized = false;
						break;
					case 98: //"b"
						displayBG = !displayBG;
						break;
					case 116: //"t"
						if (hand_status != HAND_IDENTIFYING)
						{
							hand_status = HAND_IDENTIFYING;
						}
						break;
					case 120: //"x"
						if (!tracking_interaction)
						{
							obj_model_cloud->points.clear();
							obj_Tracker.clearObjModel();
							pcl::console::print_highlight ("object cleared \n");
						}
						break;
					default:
						break;
				}
	  
			}
		}

		void 
		run ()
		{
			std::string device_id = "";
			pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber(device_id,pcl::OpenNIGrabber::OpenNI_QVGA_30Hz,pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
			boost::function<void(const pcl::visualization::KeyboardEvent&)> kb = boost::bind (&KinectTracking::keyboard_callback, this, _1);
			main_viz->registerKeyboardCallback(kb);
			boost::function<void(const RefCloudConstPtr&) > f = boost::bind (&KinectTracking::cloud_cb, this, _1);
			boost::signals2::connection c = interface->registerCallback (f);

			interface->start ();
      
			bool viewer_init = false;
			bool table_trans_added = false;
			// Loop
			while (!main_viz->wasStopped ())
			{
		
			// Render and process events in the two interactors
				main_viz->spinOnce ();

				FPS_CALC ("visualiziation");

				
				// Add the cloud
				if (filtered_cloud && cld_mutex.try_lock ())
				{
					if (!viewer_init)
					{
						main_viz->getRenderWindow ()->SetSize (1024, 768);
						main_viz->getRenderWindow ()->SetPosition (0, 0);
						main_viz->addCoordinateSystem(0.1);
						viewer_init = !viewer_init;
					}
		  
					pcl::visualization::PointCloudColorHandlerCustom<RefPointType> grey_color(g_cloud, 75, 75, 75);
					pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(obj_affordance, 150, 0, 0);
					pcl::visualization::PointCloudColorHandlerCustom<RefPointType> green_color(hand_cloud, 0, 150, 0);
					pcl::visualization::PointCloudColorHandlerCustom<RefPointType> yellow_color(hand_cloud2, 150, 150, 0);
					pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color(obj_model_cloud, 0, 0, 150);

					if (!main_viz->updatePointCloud (g_cloud,grey_color,"rawCloud"))
					{
						main_viz->addPointCloud (g_cloud, grey_color,  "rawCloud");
						main_viz->resetCameraViewpoint ("rawCloud");
					}
					
					if (!main_viz->updatePointCloud (filtered_cloud,  "filteredCloud"))
					{
						main_viz->addPointCloud (filtered_cloud,  "filteredCloud");

					}

					if (!main_viz->updatePointCloud (obj_model_cloud,blue_color,  "modelCloud"))
					{
						main_viz->addPointCloud (obj_model_cloud,blue_color,  "modelCloud");
						main_viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "modelCloud");
					}

					if (!main_viz->updatePointCloud (hand_cloud, green_color, "handCloud"))
					{
						main_viz->addPointCloud (hand_cloud, green_color, "handCloud");
						main_viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "handCloud");
					}

					if (!main_viz->updatePointCloud (hand_cloud2, yellow_color, "handCloud2"))
					{
						main_viz->addPointCloud (hand_cloud2, yellow_color, "handCloud2");
						main_viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "handCloud2");
					}

					if (!main_viz->updatePointCloud (obj_affordance, red_color, "affordance"))
					{
						main_viz->addPointCloud (obj_affordance, red_color,  "affordance");
						main_viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "affordance");
					}

	
					// draw some texts
					main_viz->removeShape ("H");
					main_viz->addText ((boost::format ("number of hand Points: %d") % hand_cloud->points.size ()).str (),
											 10, 20, 15, 1.0, 1.0, 1.0, "H");
					main_viz->removeShape ("M");
					main_viz->addText ((boost::format ("number of object Points:  %d") % obj_model_cloud->points.size ()).str (),
											 10, 40, 15, 1.0, 1.0, 1.0, "M");
					main_viz->removeShape ("A");
					main_viz->addText ((boost::format ("number of interaction Points:  %d") % obj_affordance->points.size ()).str (),
											 10, 60, 15, 1.0, 1.0, 1.0, "A");
					main_viz->removeShape ("computation");
					main_viz->addText ((boost::format ("computation time per frame:     %f ms") % (computation_time_)).str (),
											 10, 80, 15, 1.0, 1.0, 1.0, "computation");
					
 

					cld_mutex.unlock ();
		  
				}

  
			boost::this_thread::sleep (boost::posix_time::microseconds (100));

		
			}

			interface->stop ();

		}

  public:
		boost::mutex cld_mutex;
		// Create the PCLVisualizer object
		boost::shared_ptr<pcl::visualization::PCLVisualizer> main_viz;

		// Table filter
		pcl::ModelCoefficients::Ptr table_plane_coef;
		Eigen::Affine3f table_trans; 
		 
		// Create filtered point cloud object
		RefCloudPtr filtered_cloud;
		RefCloudPtr g_cloud;

		bool bg_initialized;
		bool displayBG;
		bool tracking_interaction;
		int object_status;
		int hand_status;

		// point cloud for object reference
		pcl::objTracker<RefPointType> obj_Tracker;

		RefCloudPtr obj_model_cloud;

		RefCloudPtr obj_affordance; // the parts being grasped

		// point cloud for hand
		pcl::skinFilter<RefPointType> skin_Filter;

		RefCloudPtr hand_cloud;
		Eigen::Affine3f hand_trans;
		pcl::PCA<RefPointType> hand_pca;
		Eigen::Vector3f hand_lim;

		RefCloudPtr hand_cloud2;
		

		double z_limit_;

		double computation_time_;


};


int
main (int argc, char** argv)
{
  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
    {
      if (std::string (argv[i]) == "-h")
      {
       // printHelp (argc, argv);
        return (-1);
      }
    }
  }

  //EventHelper event_helper;
  std::string device_id = "";
  pcl::console::parse_argument (argc, argv, "-dev", device_id);

  //std::string mouseMsg3D ("Mouse coordinates in PCL Visualizer");
  //std::string keyMsg3D ("Key event for PCL Visualizer");
  //cld->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg3D));    
  //cld->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg3D));
  
  KinectTracking<pcl::PointXYZRGBA> v(argc, argv);
  v.run ();
  return 0;


  
}