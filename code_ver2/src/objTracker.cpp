/*
 * Software License Agreement (BSD License)
 *
 */
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/apps/hand_obj_tracker/objTracker.h>
#include <pcl/apps/hand_obj_tracker/impl/objTracker.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(objTracker, (pcl::PointXYZRGBA)(pcl::PointXYZRGB))