#include <TableSegmenter.hpp>

using namespace pcl;

TableSegmenter::TableSegmenter()
{
  
}

PointCloud<PointXYZRGBA>::Ptr
TableSegmenter::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
  console::print_highlight ("segmenting plane...\n");
// apply color filter
// first find seed, using the middle point of the scene (assuming on the table)
  //PointXYZRGBA colorSeed = cloud_in->at(CLOUD_WIDTH/2,CLOUD_HEIGHT/2);
  //ReturnCloud.reset(new PointCloud<PointXYZRGBA>);
  /*for (size_t i = 0; i < cloud_in->points.size (); i++)
    {
      bool c_filter = (abs(colorSeed.r-cloud_in->points[i].r)<60) && 				        (abs(colorSeed.g-cloud_in->points[i].g)<60) &&						
	(abs(colorSeed.b-cloud_in->points[i].b)<60);
      if (c_filter)
	ReturnCloud->points.push_back (cloud_in->points[i]);
	}*/
			
  PointIndices::Ptr inliers (new PointIndices);
  tablePlaneCoefficient.reset(new ModelCoefficients ());
  SACSegmentation<PointXYZRGBA> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setDistanceThreshold (0.002);
  seg.setInputCloud (cloud_in);
  seg.segment (*inliers, *tablePlaneCoefficient);

// make sure the camera is always on the positive side of the reference plane
  if (tablePlaneCoefficient->values[3]<0) 
    {
        tablePlaneCoefficient->values[0] = -1.0*tablePlaneCoefficient->values[0]; 
	tablePlaneCoefficient->values[1] = -1.0*tablePlaneCoefficient->values[1];
        tablePlaneCoefficient->values[2] = -1.0*tablePlaneCoefficient->values[2]; 
	tablePlaneCoefficient->values[3] = -1.0*tablePlaneCoefficient->values[3];
	}

  projectionCloud.reset(new PointCloud<PointXYZRGBA>);
  ProjectInliers<PointXYZRGBA> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud_in);
  proj.setModelCoefficients (tablePlaneCoefficient);
  proj.filter (*projectionCloud);

  return projectionCloud;
  //return cloud_in;
}

Eigen::Affine3f TableSegmenter::ComputeTransform()
{
  Eigen::Vector4f table_c;
  compute3DCentroid(*projectionCloud,table_c);
  
  Eigen::Vector3f origin(table_c(0),table_c(1),table_c(2));
  Eigen::Vector3f zdir(tablePlaneCoefficient->values[0],
		       tablePlaneCoefficient->values[1],
		       tablePlaneCoefficient->values[2]);
  zdir.normalize();
			
  Eigen::Vector4f max_pt;
  getMaxDistance (*projectionCloud, table_c, max_pt);
  Eigen::Vector3f ydir(max_pt(0)-table_c(0),
		       max_pt(1)-table_c(1),
		       max_pt(2)-table_c(2));
  ydir.normalize();
			
  Eigen::Affine3f trans;
  getTransformationFromTwoUnitVectorsAndOrigin(ydir, 
					       zdir, 
					       origin, 
					       trans);
  return trans.inverse();
}

void TableSegmenter::PrintAlgorithm()
{
  console::print_highlight ("yo dawg this table seg nawmean \n");
}
