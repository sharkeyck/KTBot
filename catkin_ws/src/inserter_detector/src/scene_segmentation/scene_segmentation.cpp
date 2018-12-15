#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
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

#include <inserter_detector/scene_segmentation.h>

namespace inserter_detector {

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PCLPointCloud2;
using pcl::PointIndices;

PointCloud<PointXYZ>::Ptr downsample_and_convert(PCLPointCloud2::Ptr cloud) {
  pcl::VoxelGrid<PCLPointCloud2> vg;
  vg.setInputCloud(cloud);

  // leaf size of 1cm
  vg.setLeafSize (0.01f, 0.01f, 0.01f);

  PCLPointCloud2::Ptr cloud_filtered_pc2(new PCLPointCloud2);
  vg.filter(*cloud_filtered_pc2);

  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());
  pcl::fromPCLPointCloud2(*cloud_filtered_pc2, *cloud_filtered);

  return cloud_filtered;
}

std::vector<PointCloud<PointXYZ>::Ptr> extract_and_normalize_clusters(PointCloud<PointXYZ>::Ptr cloud) {
  std::vector<PointCloud<PointXYZ>::Ptr> results;

  // Setup planar segmentation
  // http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
  pcl::SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01f);
  seg.setInputCloud(cloud);
  seg.setMaxIterations(100);


  // Segment the largest planar component from the cloud (the floor)
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    return results;
  }

  // Extract the planar inlier points from the cloud and remove-them (in-place)
  pcl::ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative (true);
  extract.filterDirectly(cloud);

  // Remove NaN values (can be done in place)
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Apply reverse transform to get to world zero coordinates

  // Create the KdTree object for the search method of the extraction
  // http://pointclouds.org/documentation/tutorials/kdtree_search.php
  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud);

  // Run euclidean cluster extraction to pull out each identifiable non-floor object
  // http://docs.pointclouds.org/1.8.1/namespacepcl_1_1gpu.html#a5fe0ef2894a071171ecd36f73305259c

  std::vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 5 cm
  ec.setMinClusterSize(10); // At least 10 points
  ec.setMaxClusterSize(200); // At most 200 points
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);


  // TODO: Parallelize the next work, one thread per cluster (use threadpool?)
  // Accumulate points, find the centroid, and normalize all points so it's centered.
  // TODO: Keep track of centroids for each instance so we can understand their position when making actions
  PCLPointCloud2::Ptr normalized_cluster_pc2(new PCLPointCloud2);
  for (const PointIndices& i : cluster_indices) {
    PointCloud<PointXYZ>::Ptr cloud_cluster(new PointCloud<PointXYZ>);
    for (int j : i.indices) {
      PointXYZ p = cloud->points[j];
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
    results.push_back(normalized_cluster);
  }

  return results;
}

} // namespace inserter_detector
