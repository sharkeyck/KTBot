#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf2_msgs/TFMessage.h>
#include <inserter_detector/scene_segmentation.h>
#include <inserter_detector/CloudSegments.h>
#include <tf/tf.h>
#include <chrono>

#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define foreach BOOST_FOREACH
#include <iostream>

namespace po = boost::program_options;
using pcl::PCLPointCloud2;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using std::cout;
using std::endl;
using std::string;

using namespace inserter_detector;

void write_shard(ros::Time start, ros::Time end, string infile, string outfile, boost::asio::io_service& io_service) {
  rosbag::Bag source_bag;
  source_bag.open(infile, rosbag::bagmode::Read);
  rosbag::View view(source_bag, start, end);

  std::cout << "Writing bag: " << outfile << std::endl;
  rosbag::Bag dest_bag;
  dest_bag.open(outfile, rosbag::bagmode::Write);
  dest_bag.setChunkThreshold(768 * 1024); //bytes

  sensor_msgs::JointState::ConstPtr latest_joint_state;
  tf::Transformer trans;
  int i = 0;

  std::map<string, sensor_msgs::JointState> latest_states;
  foreach(rosbag::MessageInstance const m, view) {
    // All point cloud topics are segmented.
    // Realsense frame must exist for inverse tranformation to world coordinates
    sensor_msgs::PointCloud2::Ptr s = m.instantiate<sensor_msgs::PointCloud2>();
    if (s != NULL && trans.frameExists("/realsense")) {
      CloudSegments::Ptr s2 = extract_and_match_clusters(trans, s, "inserter", io_service);
      cloud_segment_debug(s2, i);
      // for (int i = 0; i < s2->name.size(); i++) {
      //   std::string ns = s2->name[i].substr(0, s2->name[i].find("/"));
      //   std::cout << s2->name[i] << std::endl;
      //   sensor_msgs_image_ascii(s2->image[i]);
      //   std::cout << latest_states["/" + ns + "/joint_states"] << std::endl << std::endl;
      // }
      if (s2 != NULL) {
        dest_bag.write("/inserter/cloud_segments", m.getTime(), *s2);
      }
    }

    // All joint state topics are passed through
    sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
    if (js != NULL) {
      latest_states[m.getTopic()] = *js;
      dest_bag.write(m.getTopic(), m.getTime(), js);
    }

    // Model states topics are passed through if they describe more models than the
    // next most recent model states.
    tf2_msgs::TFMessage::ConstPtr ts = m.instantiate<tf2_msgs::TFMessage>();
    if (ts != NULL) {
      for (const auto& tfs : ts->transforms) {
        tf::StampedTransform st;
        tf::transformStampedMsgToTF(tfs, st);
        trans.setTransform(st);
      }

      dest_bag.write(m.getTopic(), m.getTime(), ts);
    }
    i++;
  }

  dest_bag.close();
}

int main (int argc, char** argv) {
  string infile, outfile;
  int shards;

  try {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "show help message")
        ("input", po::value<string>(&infile)->required(), "input .bag file")
        ("output", po::value<string>(&outfile)->required(), "output .bag file")
        ("shards", po::value<int>(&shards)->required(), "number of output shards");

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
  rosbag::Bag source_bag;
  source_bag.open(infile, rosbag::bagmode::Read);

  rosbag::View view(source_bag);
  ros::Time begin_time = view.getBeginTime();
  ros::Time end_time = view.getEndTime();
  ros::Duration shard_duration = (end_time - begin_time) * (1.0/shards);

  std::cout << "(" << view.size() << " entries from " << begin_time << " to " << end_time << ")" << std::endl;

  auto t0 = std::chrono::steady_clock::now();
  std::cout << "starting io_service threads\n";
  boost::asio::io_service io_service;
  boost::thread_group threadgroup;
  boost::asio::io_service::work work(io_service);
  for (int i = 0; i < boost::thread::hardware_concurrency() - shards; ++i) {
    threadgroup.create_thread(boost::bind(&boost::asio::io_service::run,
      &io_service));
  }

  std::cout << "starting shard threads\n";
  boost::thread_group shard_threads;
  for (int i = 0; i < shards; i++) {
    std::ostringstream ss;
    ss << outfile << "." << i;
    std::string out_i = ss.str();
    shard_threads.create_thread(boost::bind(&write_shard, begin_time + shard_duration*i, begin_time + shard_duration*(i+1), infile, out_i, boost::ref(io_service)));
    // boost::thread t1();
  }
  // boost::thread t1(write_shard, v2, outfile);
  // write_shard(view, outfile);
  shard_threads.join_all();

  std::cout << std::endl << "Closing source" << std::endl;
  source_bag.close();

  auto t1 = std::chrono::steady_clock::now();
  double d0 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

  std::cout << "Done (took " << d0 << "s)" << std::endl;
}
