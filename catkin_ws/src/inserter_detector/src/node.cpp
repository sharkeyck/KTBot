#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <chrono>
#include <algorithm>
#include <std_msgs/Float64.h>

#define NODE "inserter_detector"

ros::Publisher pub;
ros::Publisher profile_pub;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PCLPointCloud2;
using pcl::PointIndices;

void cloud_cb (boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg) {
  auto start = std::chrono::steady_clock::now();

  // Convert to PCL
  PCLPointCloud2::Ptr cloud(new PCLPointCloud2);
  pcl_conversions::moveToPCL(*cloud_msg, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PCLPointCloud2> vg;
  PCLPointCloud2::Ptr cloud_filtered_pc2(new PCLPointCloud2);
  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());
  vg.setInputCloud(cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered_pc2);
  pcl::fromPCLPointCloud2(*cloud_filtered_pc2, *cloud_filtered);

  ROS_DEBUG_STREAM_THROTTLE_NAMED(10, NODE, "Most recent frame downsampled to " << float(cloud_filtered->points.size()) / cloud->data.size() * 100 << "% of original");

  // Apply pass-thru filter

  // Setup planar segmentation
  // http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
  pcl::SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01f);
  seg.setInputCloud(cloud_filtered);
  seg.setMaxIterations(100);


  // Segment the largest planar component from the cloud (the floor)
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  // Extract the planar inlier points from the cloud and remove-them (in-place)
  pcl::ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative (true);
  extract.filterDirectly(cloud_filtered);

  // Remove NaN values (can be done in place)
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);

  // Apply reverse transform to get to world zero coordinates

  // Create the KdTree object for the search method of the extraction
  // http://pointclouds.org/documentation/tutorials/kdtree_search.php
  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  // Run euclidean cluster extraction to pull out each identifiable non-floor object
  // http://docs.pointclouds.org/1.8.1/namespacepcl_1_1gpu.html#a5fe0ef2894a071171ecd36f73305259c

  std::vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 5 cm
  ec.setMinClusterSize(10); // At least 10 points
  ec.setMaxClusterSize(200); // At most 200 points
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);


  // TODO: Parallelize the next work, one thread per cluster (use threadpool?)
  // Accumulate points, find the centroid, and normalize all points so it's centered.
  // TODO: Keep track of centroids for each instance so we can understand their position when making actions
  int k = 0;
  PCLPointCloud2::Ptr normalized_cluster_pc2(new PCLPointCloud2);
  for (const PointIndices& i : cluster_indices) {
    PointCloud<PointXYZ>::Ptr cloud_cluster(new PointCloud<PointXYZ>);
    for (int j : i.indices) {
      PointXYZ p = cloud_filtered->points[j];
      cloud_cluster->points.push_back(p);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    // Normalize XYZ
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);

    PointCloud<PointXYZ>::Ptr normalized_cluster(new PointCloud<PointXYZ>);
    pcl::demeanPointCloud(*cloud_cluster, centroid, *normalized_cluster);

    pcl::toPCLPointCloud2(*normalized_cluster, *normalized_cluster_pc2);
    normalized_cluster_pc2->header = cloud->header; // Copy header to match frame etc.
    pcl_conversions::moveFromPCL(*normalized_cluster_pc2, *cloud_msg);

    k++;
  }
  ROS_INFO_STREAM_THROTTLE_NAMED(1, NODE, "Extracted " << k << " clusters");

  // Run pose estimation NN to get <base link pos, world Z angle, theta angle>

  //

  // Convert back to ROS & publish
  // pcl::toPCLPointCloud2(*cloud_filtered, *cloud_filtered_pc2);
  // pcl_conversions::moveFromPCL(*cloud_filtered_pc2, *cloud_msg);

  auto finish = std::chrono::steady_clock::now();
  pub.publish(cloud_msg);
  std_msgs::Float64 perf;
  perf.data = std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count() * 1000000;
  profile_pub.publish(perf);
}

int main (int argc, char** argv) {
  ros::init (argc, argv, NODE);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  profile_pub = nh.advertise<std_msgs::Float64>("perf", 1);
  ROS_INFO_NAMED(NODE, "Listening for point cloud data");
  ros::spin();
}
