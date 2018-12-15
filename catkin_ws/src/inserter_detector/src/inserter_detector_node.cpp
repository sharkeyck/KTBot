#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <chrono>
#include <algorithm>
#include <std_msgs/Float64.h>
#include <inserter_detector/scene_segmentation.h>

#define NODE "inserter_detector"

using namespace inserter_detector;

ros::Publisher pub;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PCLPointCloud2;
using std::string;

#define STAT_SEGMENTATION_PERF_MICROS "segmentation_perf_micros"
#define STAT_DOWNSAMPLE_RATIO "downsample_ratio"
const std::list<string> stats_fields = std::list<string>({
  STAT_SEGMENTATION_PERF_MICROS,
  STAT_DOWNSAMPLE_RATIO,
});
std::map<string, double> stats;
std::map<string, ros::Publisher> stats_publishers;

void init_stats(ros::NodeHandle& nh) {
  for (const string& f : stats_fields) {
    stats.emplace(f, 0.0);
    const string topic = "stats/" + f;
    stats_publishers.emplace(f, nh.advertise<std_msgs::Float64>(topic, 1));
  }
}

void publish_stats() {
  for (const auto& sp : stats_publishers) {
    std_msgs::Float64 val;
    val.data = stats.at(sp.first);
    sp.second.publish(val);
  }
}

void cloud_cb (boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg) {
  // Convert to PCL
  PCLPointCloud2::Ptr cloudpcl(new PCLPointCloud2);
  pcl_conversions::moveToPCL(*cloud_msg, *cloudpcl);

  // Extract clusters from the scene
  auto start = std::chrono::steady_clock::now();
  PointCloud<PointXYZ>::Ptr cloud = downsample_and_convert(cloudpcl);
  stats[STAT_DOWNSAMPLE_RATIO] = float(cloud->points.size()) / cloud_msg->data.size();

  std::vector<PointCloud<PointXYZ>::Ptr> results = extract_and_normalize_clusters(cloud);
  auto finish = std::chrono::steady_clock::now();
  stats[STAT_SEGMENTATION_PERF_MICROS] = std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count() * 1000000;

  ROS_INFO_STREAM_THROTTLE_NAMED(1, NODE, "Extracted " << results.size() << " clusters");

  // TODO: Run pose estimation NN to get <base link pos, world Z angle, theta angle>

  // Convert first detection back to ROS & publish
  if (results.size() > 0) {
    PCLPointCloud2::Ptr result_pc2(new PCLPointCloud2);
    pcl::toPCLPointCloud2(*results[0], *result_pc2);
    result_pc2->header = cloud->header; // Copy header to match frame etc.
    pcl_conversions::moveFromPCL(*result_pc2, *cloud_msg);
    pub.publish(cloud_msg);
  }

  publish_stats();
}


int main (int argc, char** argv) {
  ros::init (argc, argv, NODE);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  init_stats(nh);

  ROS_INFO_NAMED(NODE, "Listening for point cloud data");
  ros::spin();
}
