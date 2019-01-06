#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/transforms.h>
#include <chrono>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <pcl/filters/crop_box.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/future.hpp>

#include <inserter_detector/scene_segmentation.h>

namespace inserter_detector {

#define MULTI_THREAD

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointXYZ;
using pcl::PCLPointCloud2;
using pcl::PointIndices;

typedef boost::packaged_task<void> task_t;
typedef boost::shared_ptr<task_t> ptask_t;

#define IMG_SIZE 64
#define PX_PER_M (2 * IMG_SIZE)

void sensor_msgs_image_ascii(const sensor_msgs::Image& im) {
  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 64; j++) {
      std::cout << ((im.data[64*i+j] > 0) ? "- " : "  ");
    }
    std::cout << "\n";
  }
  std::cout << std::endl << std::flush;
}

void cloud_segment_debug(const CloudSegments::Ptr& s2, const int& i) {
  std::cout << "Extracted "
    << s2->name.size() << " clusters - "
    << std::fixed
    << s2->stats.pcl_conversion << " 2pcl, "
    << s2->stats.frame_extraction << " frame extraction, "
    << s2->stats.work_assignment << " work assignment, "
    << s2->stats.parallel_join << " join, "
    << (s2->stats.pcl_conversion + s2->stats.frame_extraction + s2->stats.work_assignment + s2->stats.parallel_join) << "s overall"
    << i << "\n";
}

void extract_normalized_clusters(PointCloud<PointXYZRGB>::Ptr cloud, std::vector<Segment>& results) {
  // 0.0433451s 2pcl, 0.0238638s downsample, 0.00199583s transform, 0.0388429s extract/match, 0.108048s overall

  // Setup planar segmentation
  // http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
  pcl::SACSegmentation<PointXYZRGB> seg;
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
    return;
  }

  // Extract the planar inlier points from the cloud and remove-them (in-place)
  pcl::ExtractIndices<PointXYZRGB> extract;
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
  pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);
  tree->setInputCloud(cloud);

  // Run euclidean cluster extraction to pull out each identifiable non-floor object
  // http://docs.pointclouds.org/1.8.1/namespacepcl_1_1gpu.html#a5fe0ef2894a071171ecd36f73305259c
  // TODO: Use the GPU for that
  std::vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(0.05); // 5 cm
  ec.setMinClusterSize(10); // At least 10 points
  ec.setMaxClusterSize(200); // At most 200 points
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Size results to the number of extracted clusters
  results.resize(cluster_indices.size());

  // TODO: Parallelize the next work, one thread per cluster (use threadpool?)
  // Accumulate points, find the centroid, and normalize all points so it's centered.
  // TODO: Keep track of centroids for each instance so we can understand their position when making actions
  PCLPointCloud2::Ptr normalized_cluster_pc2(new PCLPointCloud2);
  for (int i = 0; i < cluster_indices.size(); i++) {
    PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
    cloud_cluster->reserve(cluster_indices[i].indices.size());
    for (int j : cluster_indices[i].indices) {
      cloud_cluster->points.emplace_back(cloud->points[j]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    // Normalize XYZ
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);

    PointCloud<PointXYZRGB>::Ptr normalized_cluster(new PointCloud<PointXYZRGB>);
    pcl::demeanPointCloud(*cloud_cluster, centroid, *normalized_cluster);
    results[i].second = normalized_cluster;



    // Vector4f to point msg
    // http://www.pcl-users.org/converting-from-Eigen-Vector4f-to-PointT-td4025398.html
    results[i].first.x = centroid[0];
    results[i].first.y = centroid[1];
    results[i].first.z = centroid[2];
  }
}

const pcl::_PointXYZRGB* point_at(PCLPointCloud2::ConstPtr& cloudpcl, int i) {
  return reinterpret_cast<const pcl::_PointXYZRGB*>(&(cloudpcl->data[i * cloudpcl->point_step]));
}

inline float point_distance(float x1, float y1, float x2, float y2) {
  return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}

void cluster_worker(PCLPointCloud2::ConstPtr cloudpcl, const geometry_msgs::Point& centroid, sensor_msgs::Image* dest) {
  // TODO make this configurable
  const double BOUND = 0.4;
  const float width = cloudpcl->width, height = cloudpcl->height;

  float size = cloudpcl->height * cloudpcl->width * cloudpcl->point_step;
  const pcl::_PointXYZRGB *first;
  first = reinterpret_cast<const pcl::_PointXYZRGB*>(&(cloudpcl->data[0]));
  const pcl::_PointXYZRGB *last;
  last = reinterpret_cast<const pcl::_PointXYZRGB*>(&(cloudpcl->data[size - cloudpcl->point_step]));

  const float real_width = abs(last->x - first->x);
  const float real_height = abs(last->y - first->y);
  int bbox_width = abs(round(width / real_width * BOUND));
  int bbox_height = abs(round(height / real_height * BOUND));

  // Locate the nearest point to the centroid in X and Y.
  // This is a crappy 2d binary search, as we can assume the
  // camera data is at least somewhat organized even with a transform applied.
  // TODO: do in main thread?
  // int bomin[2] = {0,0};
  // int bomax[2] = {int(width)-1,int(height)-1};
  // while (bomin[0] < bomax[0] && bomin[1] < bomax[1]) {
  //   const pcl::_PointXYZRGB* pmin = point_at(cloudpcl, bomin[1]*width + bomin[0]);
  //   const pcl::_PointXYZRGB* pmax = point_at(cloudpcl, bomax[1]*width + bomax[0]);
  //   float sdmin = point_distance(pmin->x, pmin->y, centroid.x, centroid.y);
  //   float sdmax = point_distance(pmax->x, pmax->y, centroid.x, centroid.y);
  //   if (sdmin < sdmax) {
  //     bomax[2] = (bomax[2]-bomin[2])/2;
  //     bomax[1] = (bomax[1]-bomin[1])/2;
  //   } else {
  //     bomax[2] = (bomax[2]-bomin[2])/2;
  //     bomax[1] = (bomax[1]-bomin[1])/2;
  //   }
  // }

  int box_origin_x = -1;
  int box_origin_y = -1;
  float closest_sd = 100000;
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      const pcl::_PointXYZRGB* p = point_at(cloudpcl, i*width + j);
      float dx = (p->x - centroid.x);
      float dy = (p->y - centroid.y);
      float sd = dx*dx+dy*dy;
      if (sd < closest_sd) {
        box_origin_y = i;
        box_origin_x = j;
        closest_sd = sd;
      }
    }
  }

  // const pcl::_PointXYZRGB* ptest = point_at(cloudpcl, box_origin_y*width + box_origin_x);
  //std::cout << dest->encoding << " " << box_origin_x << "," << box_origin_y << " - " << ptest->x << ", " << ptest->y << std::endl;

  // Construct point indices
  // TODO: Just iterate regularly
  std::vector<int> indices;
  indices.reserve(bbox_width*bbox_height);
  for (int i = box_origin_y - bbox_height/2; i < box_origin_y + bbox_height/2; i++) {
    for (int j = box_origin_x - bbox_width/2; j < box_origin_x + bbox_width/2; j++) {
      int offs = i * width + j;
      if (offs < 0 || offs > width*height) {
        continue;
      }
      // Basic floor removal (TODO make more robust)
      const pcl::_PointXYZRGB* p = point_at(cloudpcl, i*width + j);
      if (p->z < 0.01 || p->z > 2.99) {
        continue;
      }
      indices.push_back(offs);
    }
  }
  if (indices.size() == 0) {
    std::cerr << "Zero indices" << std::endl << std::flush;
  }

  // Write destination image
  dest->encoding = "mono8";
  dest->height = IMG_SIZE;
  dest->width = IMG_SIZE;
  dest->step = IMG_SIZE;
  dest->data.resize(IMG_SIZE*IMG_SIZE);
  std::fill(dest->data.begin(), dest->data.end(), 0);

  // PointCloud<PointXYZRGB>::Ptr full_cloud(new PointCloud<PointXYZRGB>());
  // pcl::fromPCLPointCloud2(*cloudpcl, *full_cloud);

  float minZ = 100000, maxZ = -100000;
  for (const auto& i : indices) {
    const pcl::_PointXYZRGB *p = point_at(cloudpcl, i);
    minZ = std::min(minZ, p->z);
    maxZ = std::max(maxZ, p->z);
  }

  if (fabs(maxZ - minZ) < 0.01) {
    std::cerr << "WARNING: Max and min z values are close: " << minZ << " - " << maxZ << std::endl << std::flush;
  }

  for (const auto& i : indices) {
    const pcl::_PointXYZRGB *p = point_at(cloudpcl, i);
    float tx = p->x;
    float ty = p->y;
    // std::cout << tx << ", " << ty << std::endl;
    // Orient coordinates: x left-to-right, y bottom-to-top of image
    int x = (p->x - centroid.x) * PX_PER_M + IMG_SIZE/2;
    int y = - (p->y - centroid.y) * PX_PER_M + IMG_SIZE/2;
    int offs = y * IMG_SIZE + x;
    if (offs < 0 || offs >= IMG_SIZE*IMG_SIZE) {
      continue;
    }

    // Z value becomes intensity, normalized 0-255
    uint8_t v = uint8_t(round((p->z - minZ) / (maxZ - minZ) * 255));
    dest->data[offs] = v;
  }
  return;
}

CloudSegments::Ptr extract_and_match_clusters(const tf::Transformer& trans, const sensor_msgs::PointCloud2::Ptr cloud_msg, const std::string& frame_filter, boost::asio::io_service& io_service) {
  // Translate to world frame
  // and convert to PCL
  // (this is a rotation around the Y axis)
  auto t0 = std::chrono::steady_clock::now();
  tf::StampedTransform tfw;
  trans.lookupTransform(cloud_msg->header.frame_id, "world", ros::Time(0), tfw);
  sensor_msgs::PointCloud2 world_cloud;
  pcl_ros::transformPointCloud("world", tfw, *cloud_msg, world_cloud);
  PCLPointCloud2::Ptr cloudpcl(new PCLPointCloud2);
  pcl_conversions::moveToPCL(world_cloud, *cloudpcl);

  const auto p1 = reinterpret_cast<const pcl::_PointXYZRGB*>(&(cloud_msg->data[cloud_msg->point_step * 640*470]));
  const auto p2 = reinterpret_cast<const pcl::_PointXYZRGB*>(&(cloudpcl->data[cloud_msg->point_step * 640*470]));
  // std::cout << p1->x << " -> " << p2->x << ", " << p1->y << " -> " << p2->y << std::endl;

  // Get frames locating the models
  auto t1 = std::chrono::steady_clock::now();
  std::vector<std::string> model_frames; // = {"inserter0/base_link", "inserter3/base_link", "inserter12/base_link", "inserter15/base_link"};
  // for (const auto& s : model_frames) {
  //   if (!trans.frameExists(s)) {
  //     CloudSegments::Ptr result(new CloudSegments());
  //     return result;
  //   }
  // }
  trans.getFrameStrings(model_frames);

  model_frames.erase(std::remove_if(model_frames.begin(), model_frames.end(), [&frame_filter](const std::string& el){
    return el.find(frame_filter) == std::string::npos;
  }), model_frames.end());

  int n = model_frames.size();


  auto t2 = std::chrono::steady_clock::now();
  CloudSegments::Ptr result(new CloudSegments());
  result->name.resize(n);
  result->image.resize(n);
  result->centroid.resize(n);

  // Loop through frames, assigning name and centroid and starting a worker thread to handle the extraction
  std::vector<boost::unique_future<void>> pending_data(n);
  for (int i = 0; i < n; i++) {
    result->name[i] = model_frames[i];
    tf::StampedTransform tfi;
    trans.lookupTransform("/world", model_frames[i], ros::Time(0), tfi);
    auto point = tfi.getOrigin();

    // TODO TODO TODO - I don't know why this transformation works, and it worries me.
    // But if I don't do this, the centroids are not correctly assigned.
    // Can debug this using a non-symmetric positioning of robots (e.g. 3 of them in an L shape)
    result->centroid[i].x = point.y();
    result->centroid[i].y = -point.x();
    result->centroid[i].z = point.z();

    // std::cout << result->name[i] << std::endl;
    // std::cout << point.x() << " " << point.y() << std::endl;

    result->image[i].encoding = result->name[i]; // side channel

#ifndef MULTI_THREAD
    cluster_worker(cloudpcl, result->centroid[i], &(result->image[i]));
#endif
#ifdef MULTI_THREAD
    ptask_t task = boost::make_shared<task_t>(boost::bind(&cluster_worker, cloudpcl, result->centroid[i], &(result->image[i])));
    pending_data[i] = task->get_future();
    io_service.post(boost::bind(&task_t::operator(), task));
#endif
  }
  auto t3 = std::chrono::steady_clock::now();
#ifdef MULTI_THREAD
  boost::wait_for_all(pending_data.begin(), pending_data.end());
#endif
  auto t4 = std::chrono::steady_clock::now();
  result->stats.pcl_conversion = ros::Duration(std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count());
  result->stats.frame_extraction = ros::Duration(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
  result->stats.work_assignment = ros::Duration(std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count());
  result->stats.parallel_join = ros::Duration(std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count());
  return result;
}

} // namespace inserter_detector
