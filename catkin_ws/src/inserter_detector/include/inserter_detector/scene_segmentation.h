#ifndef INSERTER_DETECTOR_SCENE_SEGMENTATION_H
#define INSERTER_DETECTOR_SCENE_SEGMENTATION_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

namespace inserter_detector {

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_and_convert(pcl::PCLPointCloud2::Ptr cloud);

// Remove the planar floor surface and extract a number of clusters from the point cloud scene, shifting them
// so their centroids are at <0,0,0>.
// The input cloud is modified in the process.
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extract_and_normalize_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

} // namespace inserter_detector

#endif // INSERTER_DETECTOR_SCENE_SEGMENTATION_H
