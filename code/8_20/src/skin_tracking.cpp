/*
 * Software License Agreement (BSD License)
 *
 */
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/apps/hand_obj_tracker/skin_tracking.h>
#include <pcl/apps/hand_obj_tracker/impl/skin_tracking.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(skin_tracking, (pcl::PointXYZRGBA)(pcl::PointXYZRGB))



