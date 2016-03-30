/*
 * Software License Agreement (BSD License)
 *
 */

#ifndef PCL_OBJ_TRACKER_H_
#define PCL_OBJ_TRACKER_H_

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

#include <Eigen/Core>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/pcl_search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl::tracking;

namespace pcl
{
	template <typename PointT>
	class objTracker :	public PCLBase<PointT>
	{
		protected:
      using PCLBase<PointT>::deinitCompute;

		public:
			using PCLBase<PointT>::input_;

			typedef boost::shared_ptr< objTracker<PointT> > Ptr;
      typedef boost::shared_ptr< const objTracker<PointT> > ConstPtr;

			typedef pcl::octree::OctreePointCloudSearch<PointT> OctreeSearch;
      typedef boost::shared_ptr <OctreeSearch> OctreeSearchPtr;
      typedef boost::shared_ptr <const OctreeSearch> OctreeSearchConstPtr;

			typedef pcl::PointNormal PointNT;
			typedef ParticleXYZRPY ParticleT;
			typedef pcl::PointCloud<PointT> PointCloud;
			typedef pcl::PointCloud<PointNT> NormalCloud;

      typedef typename PointCloud::Ptr PointCloudPtr;
			typedef typename NormalCloud::Ptr NormalCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

			//typedef KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> ParticleFilter;
			typedef ParticleFilterOMPTracker<PointT, ParticleT> ParticleFilter;
			typedef typename ParticleFilter::CoherencePtr CoherencePtr;
			typedef typename pcl::search::KdTree<PointT> KdTree;
			typedef typename KdTree::Ptr KdTreePtr;

			/** \brief Empty constructor. */
			objTracker () :  modelExisted(false), thread_nr(4)
				,downsampling_grid_size_(0.01)
			{
				obj_model_cloud.reset(new PointCloud);
				obj_tracked_cloud.reset(new PointCloud);
				// setup octree collision detector
				obj_octree.reset(new OctreeSearch(32.0f));
				obj_model_normals.reset(new NormalCloud);
				// setup normal estimator
				normalEstim.setNumberOfThreads(thread_nr);
				KdTreePtr tree (new KdTree (false));
				normalEstim.setSearchMethod (tree);
				normalEstim.setRadiusSearch (0.05);

				// setup tracker
				std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
				default_step_covariance[3] *= 40.0;
				default_step_covariance[4] *= 40.0;
				default_step_covariance[5] *= 40.0;

				std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
				std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

				tracker_.reset(new ParticleFilter(thread_nr)); //OMP

				tracker_->setTrans (Eigen::Affine3f::Identity ());
				tracker_->setStepNoiseCovariance (default_step_covariance);
				tracker_->setInitialNoiseCovariance (initial_noise_covariance);
				tracker_->setInitialNoiseMean (default_initial_mean);
				tracker_->setIterationNum (1);
    
				tracker_->setParticleNum (250);
				tracker_->setResampleLikelihoodThr(0.00);
				tracker_->setUseNormal (false);
				// setup coherences
				ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence = ApproxNearestPairPointCloudCoherence<PointT>::Ptr
					(new ApproxNearestPairPointCloudCoherence<PointT> ());
    
				boost::shared_ptr<DistanceCoherence<PointT> > distance_coherence
					= boost::shared_ptr<DistanceCoherence<PointT> > (new DistanceCoherence<PointT> ());
				coherence->addPointCoherence (distance_coherence);
    
				boost::shared_ptr<HSVColorCoherence<PointT> > color_coherence
					= boost::shared_ptr<HSVColorCoherence<PointT> > (new HSVColorCoherence<PointT> ());
				color_coherence->setWeight (0.1);
				coherence->addPointCoherence (color_coherence);
    
				//boost::shared_ptr<pcl::search::KdTree<PointT> > search (new pcl::search::KdTree<PointT> (false));
				boost::shared_ptr<pcl::search::Octree<PointT> > search (new pcl::search::Octree<PointT> (0.01));
				coherence->setSearchMethod (search);
				coherence->setMaximumDistance (0.01);
				tracker_->setCloudCoherence (coherence);
				
			};

			/** \brief Empty destructor. */
			virtual ~objTracker() {};

			/** \brief Load and process the cloud in the given PCD file.  */
			inline int
			loadObjModelCloud (const std::string &pcd_file)
			{
				obj_model_cloud.reset(new PointCloud);
				return( pcl::io::loadPCDFile (pcd_file, *obj_model_cloud) );
			};

			/** \brief Clear object model. */
			inline void
			clearObjModel () {obj_model_cloud.reset(new PointCloud);}

			/** \add new samples*/
			bool 
			addSamples (const PointCloudConstPtr &cloud_in);

			/** \ normal estimation with OMP  */
			inline void
			normalEstimation (const PointCloudConstPtr &cloud_in, NormalCloud &result)
			{
				normalEstim.setInputCloud (cloud_in);
				normalEstim.compute (result);
			};

			/** \ grid down sampling  */
			inline void 
			gridSampleApprox (const PointCloudConstPtr &cloud_in, PointCloud &result, double leaf_size = 0.01)
			{
				pcl::ApproximateVoxelGrid<PointT> grid;
				grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
				grid.setInputCloud (cloud_in);
				grid.filter (result);
			};

			/** \ get object model cloud */
			inline PointCloudPtr const
			getObjCloud() { return (obj_tracked_cloud);}

			/** \ setup collision detection threshold*/
			//inline void 
			//setCollisionTH(const float th) {collisionTH = th;}

			/** \ setup octree for collision */
			bool 
			initializeCollision (void);

			/** \ do collision detection*/
			int
			collisionDetect (const PointCloudConstPtr &cloud_in, PointCloud &collision_cloud);

			/** \ setup reference for tracking */
			inline bool 
			initializeTracking ()
			{
				if (obj_model_cloud->points.empty())
				{ 
					return false;
				}
				else
				{
					downsampling_grid_size_ = 0.01 * static_cast<double> (obj_model_cloud->points.size())/1500;
					PointCloudPtr transed_ref (new PointCloud);
					pcl::transformPointCloud<PointT> (*obj_model_cloud, *transed_ref, obj_model_trans.inverse());
					obj_model_cloud = transed_ref;
					PointCloudPtr downsampled_model (new PointCloud);
					gridSampleApprox (transed_ref, *downsampled_model, downsampling_grid_size_);
					tracker_->setReferenceCloud (downsampled_model);
					tracker_->setTrans (obj_model_trans);
          tracker_->setMinIndices (int (obj_model_cloud->points.size ()) / 2);
				}
			};

			/** \ do tracking */
			inline bool
			updatePoseFromInputCloud ()
			{
				PointCloudPtr downsampled_input (new PointCloud);
				gridSampleApprox (input_, *downsampled_input, downsampling_grid_size_);
				tracker_->setInputCloud (downsampled_input);
				tracker_->compute ();
				ParticleXYZRPY result = tracker_->getResult ();
				obj_model_trans = tracker_->toEigenMatrix (result);
				pcl::transformPointCloud<PointT> (*obj_model_cloud, *obj_tracked_cloud, obj_model_trans);
			}
			

		protected:
			bool modelExisted;
			int thread_nr;

			PointCloudPtr obj_model_cloud;
			PointCloudPtr obj_tracked_cloud;
			NormalCloudPtr obj_model_normals;
			
			Eigen::Affine3f obj_model_trans;			

			pcl::NormalEstimationOMP<PointT, PointNT> normalEstim;

			// for collision
			OctreeSearchPtr obj_octree;

			double downsampling_grid_size_;

			boost::shared_ptr<ParticleFilter> tracker_;

			
			
	};

}


#endif  //#ifndef PCL_OBJ_TRACKER_H_