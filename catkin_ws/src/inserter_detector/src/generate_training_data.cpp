#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <inserter_detector/scene_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <iostream>

rosbag::Bag source_bag;
rosbag::Bag dest_bag;

using pcl::PCLPointCloud2;
using pcl::PointCloud;
using pcl::PointXYZ;

using namespace inserter_detector;

boost::shared_ptr<sensor_msgs::PointCloud2> convert_cloud(boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg) {
  // Convert to PCL
  PCLPointCloud2::Ptr cloudpcl(new PCLPointCloud2);
  pcl_conversions::moveToPCL(*cloud_msg, *cloudpcl);

  // Extract clusters from the scene
  PointCloud<PointXYZ>::Ptr cloud = downsample_and_convert(cloudpcl);
  std::vector<PointCloud<PointXYZ>::Ptr> results = extract_and_normalize_clusters(cloud);

  // Convert first detection back to ROS & publish
  if (results.size() > 0) {
    PCLPointCloud2::Ptr result_pc2(new PCLPointCloud2);
    pcl::toPCLPointCloud2(*results[0], *result_pc2);
    result_pc2->header = cloud->header; // Copy header to match frame etc.

    boost::shared_ptr<sensor_msgs::PointCloud2> result;
    pcl_conversions::moveFromPCL(*result_pc2, *result);
    return result;
  }

  return NULL;
}

int main (int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: gen_training_data <path_to_bag_file> <output_path>" << std::endl;
    return 1;
  }

  std::cout << "Reading bag: " << argv[1] << std::endl;
  source_bag.open(argv[1], rosbag::bagmode::Read);
  std::cout << "Writing bag: " << argv[1] << std::endl;
  dest_bag.open(argv[2], rosbag::bagmode::Write);
  std::vector<std::string> topics = {"/realsense/depth_registered/points", "/inserter/joint_states"};
  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  sensor_msgs::JointState::ConstPtr latest_joint_state;
  foreach(rosbag::MessageInstance const m, view) {
    boost::shared_ptr<sensor_msgs::PointCloud2> s = m.instantiate<sensor_msgs::PointCloud2>();
    if (s != NULL) {
      auto s2 = convert_cloud(s);
      if (s2 != NULL) {
        dest_bag.write(topics[0], m.getTime(), s2);
      }
    }

    sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
    if (js != NULL) {
      dest_bag.write(topics[1], m.getTime(), js);
    }

    std::cout << ".";
  }

  std::cout << std::endl << "Closing bags" << std::endl;
  source_bag.close();
  dest_bag.close();
  std::cout << "Done" << std::endl;
}
