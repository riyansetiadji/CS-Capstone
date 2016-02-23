/*
 * Software License Agreement (BSD License)
 *
 */

#ifndef PCL_OBJ_TRACKER_H_
#define PCL_OBJ_TRACKER_H_

#include <pcl/pcl_base.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/pcl_search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>

namespace pcl
{
	template <typename PointT>
	class objTracker :	public PCLBase<PointT>
	{
		public:
			using PCLBase<PointT>::input_;

			typedef pcl::PointNormal PointNT;
			typedef pcl::PointCloud<PointT> PointCloud;
			typedef pcl::PointCloud<PointNT> NormalCloud;
			typedef pcl::FPFHSignature33 FeatureT;
			typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
			typedef pcl::PointCloud<FeatureT> FeatureCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
			typedef typename NormalCloud::Ptr NormalCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

			typedef pcl::octree::OctreePointCloudSearch<PointT> OctreeSearch;
      typedef boost::shared_ptr <OctreeSearch> OctreeSearchPtr;
      typedef boost::shared_ptr <const OctreeSearch> OctreeSearchConstPtr;
      
			/** \brief Empty constructor. */
			objTracker () :  modelExisted(false)
			{
				obj_model_cloud.reset(new PointCloud);
				// setup octree collision detector
				obj_octree.reset(new OctreeSearch(32.0f));
				obj_model_normals.reset(new NormalCloud);
				// setup normal estimator
				normalEstim.setNumberOfThreads(4);
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> (false));
				normalEstim.setSearchMethod (tree);
				normalEstim.setRadiusSearch (0.05);
			}

			/** \brief Empty destructor. */
			virtual ~objTracker() {};

			/** \brief Load and process the cloud in the given PCD file.  */
			inline int
			loadObjModelCloud (const std::string &pcd_file)
			{
				obj_model_cloud.reset(new PointCloud);
				return( pcl::io::loadPCDFile (pcd_file, *obj_model_cloud) );
			}

			/** \brief Clear object model. */
			inline void
			clearObjModel () {obj_model_cloud.reset(new PointCloud);}

			/** \add new samples*/
			bool 
			addSamples (const PointCloudConstPtr &cloud_in);

			/** \ normal estimation with OMP  */
			inline void
			normalEstimation (const PointCloudConstPtr &cloud, NormalCloud &result)
			{
				normalEstim.setInputCloud (cloud);
				normalEstim.compute (result);
			}

			/** \ get object model cloud */
			inline PointCloudPtr const
			getObjCloud() { return (obj_model_cloud);}

			/** \ setup collision detection threshold*/
			//inline void 
			//setCollisionTH(const float th) {collisionTH = th;}

			/** \ setup octree for collision */
			bool 
			initializeCollision (void);

			/** \ do collision detection*/
			int
			collisionDetect (const PointCloudConstPtr &cloud_in, PointCloud &collision_cloud);




		protected:
			bool modelExisted;

			PointCloudPtr obj_model_cloud;
			NormalCloudPtr obj_model_normals;
			
			Eigen::Affine3f obj_model_trans;			

			pcl::NormalEstimationOMP<PointT, PointNT> normalEstim;

			// for collision
			OctreeSearchPtr obj_octree;
			
	};

}


#endif  //#ifndef PCL_OBJ_TRACKER_H_