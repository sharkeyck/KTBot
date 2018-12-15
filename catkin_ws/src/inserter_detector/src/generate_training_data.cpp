#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <inserter_detector/scene_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/progress.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <iostream>

rosbag::Bag source_bag;
rosbag::Bag dest_bag;

namespace po = boost::program_options;
using pcl::PCLPointCloud2;
using pcl::PointCloud;
using pcl::PointXYZ;
using std::cout;
using std::endl;
using std::string;

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

    boost::shared_ptr<sensor_msgs::PointCloud2> result(new sensor_msgs::PointCloud2());
    pcl_conversions::moveFromPCL(*result_pc2, *result);
    return result;
  }

  return NULL;
}

int main (int argc, char** argv) {
  string infile, outfile;

  try {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "show help message")
        ("input", po::value<string>(&infile)->required(), "input .bag file")
        ("output", po::value<string>(&outfile)->required(), "output .bag file");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    po::notify(vm);

  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << "\n";
      return 2;
  }


  std::cout << "Reading bag: " << infile;
  source_bag.open(infile, rosbag::bagmode::Read);

  std::vector<std::string> topics = {
    "/realsense/depth_registered/points",
    "/inserter/joint_states"
  };
  rosbag::View view(source_bag, rosbag::TopicQuery(topics));
  std::cout << "(" << view.size() << " entries)" << std::endl;

  std::cout << "Writing bag: " << outfile << std::endl;
  dest_bag.open(outfile, rosbag::bagmode::Write);

  boost::progress_display show_progress(view.size());

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

    ++show_progress;
  }

  std::cout << std::endl << "Closing bags" << std::endl;
  source_bag.close();
  dest_bag.close();
  std::cout << "Done" << std::endl;
}
