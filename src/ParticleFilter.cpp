#include <ParticleFilter.hpp>
#include <TrackerManager.hpp>

using namespace pcl;
using namespace std;

ParticleFilter::ParticleFilter(std::string ff, std::string d_id)
{
  filename = ff;
  device_id = d_id;
  counter = 0;
  new_cloud_ = false;
  model_flag = false;

  downsampling_grid_size_ =  0.01;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 1.1);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  boost::shared_ptr<tracking::KLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY> > tracker
    (new tracking::KLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY> (8));

  tracking::ParticleXYZRPY bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;


  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (100);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.1);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (100);
  tracker_->setResampleLikelihoodThr(0.00);
  //tracker_->setUseNormal (true);
  tracker_->setUseNormal (false);

  //Setup coherence object for tracking
tracking::ApproxNearestPairPointCloudCoherence<PointXYZRGBNormal>::Ptr coherence = tracking::ApproxNearestPairPointCloudCoherence<PointXYZRGBNormal>::Ptr
    (new tracking::ApproxNearestPairPointCloudCoherence<PointXYZRGBNormal> ());

  boost::shared_ptr<tracking::DistanceCoherence<PointXYZRGBNormal> > distance_coherence
    = boost::shared_ptr<tracking::DistanceCoherence<PointXYZRGBNormal> > (new tracking::DistanceCoherence<PointXYZRGBNormal> ());
  coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<tracking::NormalCoherence<PointXYZRGBNormal> > normal_coherence
    = boost::shared_ptr<tracking::NormalCoherence<PointXYZRGBNormal> > (new tracking::NormalCoherence<PointXYZRGBNormal> ());
    normal_coherence->setWeight(100);
    //coherence->addPointCoherence (normal_coherence);

  boost::shared_ptr<search::Octree<PointXYZRGBNormal> > search (new search::Octree<PointXYZRGBNormal> (0.1));
  coherence->setSearchMethod (search);
  //coherence->setMaximumDistance (1.01);

  tracker_->setCloudCoherence (coherence);

}

PointCloud<PointXYZRGBA>::Ptr
ParticleFilter::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  PointCloud<PointXYZRGBNormal>::Ptr cloudy = getCloudNormals(cloud_in);
  //PointCloud<PointXYZRGBNormal>::Ptr cloudy (new PointCloud<PointXYZRGBNormal>);

  if(!model_flag)
{
  PointCloud<PointXYZRGBA>::Ptr modelCloud ( new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr modelCloudDownsampled ( new PointCloud<PointXYZRGBA>);

    if(io::loadPCDFile (filename, *modelCloud) == -1)
    {
      std::cout << "pcd file not found" << std::endl;
    }
    else
      {
	std::cout << "Model loaded" << std::endl;
      }

    GridSampleApprox (modelCloud, *modelCloudDownsampled, downsampling_grid_size_);
    target_cloud.reset(new PointCloud<PointXYZRGBNormal>());
    target_cloud = getCloudNormals(modelCloudDownsampled);
    
    setModel(target_cloud);
    model_flag = true;
  }

  //std::cout << cloud_in->points.size();
  //cloud_pass_.reset (new PointCloud<PointXYZRGBNormal>);
  //cloud_pass_downsampled_.reset (new PointCloud<PointXYZRGBNormal>);
  //FilterPassThrough (cloud_in, *cloud_pass_);
  //FilterPassThrough (cloudy, *cloud_pass_);
  //GridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  if(counter < 10)
  {
	counter++;
  }
  else if(!cloud_in->empty())
  {
    //Track the object
    //std::cout << cloud_pass_downsampled_->points.size() + "\n";
    tracker_->setInputCloud (cloudy);
    //tracker_->setInputCloud (cloud_in);
    tracker_->compute ();
    new_cloud_ = true;
  }

  DrawParticles();
  ReturnCloud = cloud_in;
  /*ReturnCloud.reset(new PointCloud<PointXYZRGBA>);
  for(int i = 0; i < norm_cloud->points.size();i++)
    {
      PointXYZRGB point;
      point.x = norm_cloud->points[i].x;
      point.y = norm_cloud->points[i].y;
      point.z = norm_cloud->points[i].z;
      point.r = norm_cloud->points[i].r;
      point.g = norm_cloud->points[i].g;
      point.b = norm_cloud->points[i].b;
      rgbNorm->points.push_back(point);
      }*/
  return ReturnCloud;
}

void ParticleFilter::GridSampleApprox(const PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointCloud<PointXYZRGBA> &result, double leaf_size)
{
  ApproximateVoxelGrid<PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void ParticleFilter::FilterPassThrough(const PointCloud<PointXYZRGBNormal>::Ptr &cloud, PointCloud<PointXYZRGBNormal> &result)
{
  PassThrough<PointXYZRGBNormal> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, TrackerManager::GlobalTracker()->GetZDepth());
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

bool ParticleFilter::DrawParticles()
{
  tracking::ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  PointCloud<PointXYZRGBNormal>::Ptr result_cloud (new PointCloud<PointXYZRGBNormal>);
  pcl::transformPointCloud<PointXYZRGBNormal> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBNormal> blue_color (result_cloud, 255, 255, 5);
boost::shared_ptr<visualization::PCLVisualizer> visualizer = 
    TrackerManager::GlobalTracker()->GetVisualizer();
 std::string name = "pringles curley mustache";
  if (!visualizer->updatePointCloud (result_cloud, blue_color, name))
    {
      visualizer->addPointCloud (result_cloud, blue_color, name);
      visualizer->setPointCloudRenderingProperties
	(visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
    }
}

Eigen::Affine3f ParticleFilter::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void ParticleFilter::PrintAlgorithm()
{
  
}
void ParticleFilter::setModel(const PointCloud<PointXYZRGBNormal>::Ptr &cloud_in)
{
  
  //prepare the model of tracker's target
  /*Eigen::Vector4f c;
  Eigen::Vector4f d;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  Eigen::Affine3f modelTrans = Eigen::Affine3f::Identity ();
  PointCloud<PointXYZRGBNormal>::Ptr transed_ref (new PointCloud<PointXYZRGBNormal>);

  compute3DCentroid<PointXYZRGBNormal> (*cloud_in, c);
  compute3DCentroid<PointXYZRGBNormal> (*target_cloud, d);

  Eigen::Vector4f e = c-d;

  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  modelTrans.translation().matrix()= Eigen::Vector3f (e[0], e[1], e[2]); 
  transformPointCloud<PointXYZRGBNormal> (*target_cloud, *transed_ref, modelTrans);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref);*/
  tracker_->setReferenceCloud (target_cloud);
  //tracker_->setTrans (trans);
}

PointCloud<PointXYZRGBNormal>::Ptr 
ParticleFilter::getCloudNormals(const PointCloud<PointXYZRGBA>::Ptr &norm_cloud)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointXYZRGBA, Normal> ne;
  ne.setInputCloud (norm_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<PointXYZRGBA> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<Normal>::Ptr cloud_normals (new pcl::PointCloud<Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  PointCloud<PointXYZRGBNormal>::Ptr pointNormals (new PointCloud<PointXYZRGBNormal>);
  
  PointCloud<PointXYZRGB>::Ptr rgbNorm (new PointCloud<PointXYZRGB>);
  for(int i = 0; i < norm_cloud->points.size();i++)
    {
      PointXYZRGB point;
      point.x = norm_cloud->points[i].x;
      point.y = norm_cloud->points[i].y;
      point.z = norm_cloud->points[i].z;
      point.r = norm_cloud->points[i].r;
      point.g = norm_cloud->points[i].g;
      point.b = norm_cloud->points[i].b;
      rgbNorm->points.push_back(point);
    }
  
  //pointNormals += cloud_normals;

  //concatenateFields<PointXYZRGB, Normal, PointXYZRGBNormal>(rgbNorm, cloud_normals, pointNormals);
  concatenateFields(*rgbNorm, *cloud_normals, *pointNormals);
  //concatenatePointCloud(norm_cloud, cloud_normals, pointNormals);
  
  // io::savePCDFileBinary ("../data/normal_data/bbq.pcd", *target_cloud);
  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

  return pointNormals;
}
