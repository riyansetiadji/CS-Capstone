/*
 * Software License Agreement (BSD License)
 *
 */
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/apps/hand_obj_tracker/skinFilter.h>
#include <pcl/apps/hand_obj_tracker/impl/skinFilter.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(skinFilter, (pcl::PointXYZRGBA)(pcl::PointXYZRGB))



