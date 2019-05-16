#ifndef INSERTER_DETECTOR_SCENE_SEGMENTATION_H
#define INSERTER_DETECTOR_SCENE_SEGMENTATION_H

#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <inserter_detector/CloudSegments.h>
#include <boost/asio.hpp>

namespace inserter_detector {

void sensor_msgs_image_ascii(const sensor_msgs::Image& im);

void cloud_segment_debug(const CloudSegments::Ptr& s2, const int& i);

// Includes a centroid position and a demeaned point cloud
typedef std::pair<pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Segment;

// The input cloud is modified in the process.
void extract_normalized_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Segment>& results);

// Remove the planar floor surface and extract a number of clusters from the point cloud scene, shifting them
// so their centroids are at <0,0,0>.
CloudSegments::Ptr extract_and_match_clusters(const tf::Transformer& trans, const sensor_msgs::PointCloud2::Ptr cloud_msg, const std::string& frame_filter, boost::asio::io_service& io_service);

} // namespace inserter_detector

#endif // INSERTER_DETECTOR_SCENE_SEGMENTATION_H
