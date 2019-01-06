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
#include <inserter_detector/CloudSegments.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define NODE "inserter_detector"

using namespace inserter_detector;

ros::Publisher pub;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PCLPointCloud2;
using std::string;

tf::TransformListener *listener;
boost::asio::io_service io_service;

void cloud_cb (sensor_msgs::PointCloud2::Ptr cloud_msg) {
  if (listener->frameExists("/realsense")) {
    auto start = std::chrono::steady_clock::now();
    CloudSegments::Ptr results = extract_and_match_clusters(*listener, cloud_msg, "inserter", io_service);
    pub.publish(*results);
    auto finish = std::chrono::steady_clock::now();
    ROS_INFO_STREAM_THROTTLE_NAMED(1, NODE, "Extracted " << results->name.size() << " clusters");
  }
}

int main (int argc, char** argv) {
  ros::init (argc, argv, NODE);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
  pub = nh.advertise<inserter_detector::CloudSegments>("output", 1);

  listener = new tf::TransformListener();

  std::cout << "starting io_service threads\n";
  boost::thread_group threadgroup;
  boost::asio::io_service::work work(io_service);
  for (int i = 0; i < boost::thread::hardware_concurrency(); ++i) {
    threadgroup.create_thread(boost::bind(&boost::asio::io_service::run,
      &io_service));
  }

  ROS_INFO_NAMED(NODE, "Listening for point cloud data");
  ros::spin();
}
