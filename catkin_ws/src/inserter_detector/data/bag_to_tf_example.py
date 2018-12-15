import rosbag

pc_topic = "/realsense/depth_registered/points"
tf_topic = "/TODO"

bag = rosbag.Bag('inserter_sweep_single.bag')
for topic, msg, t in bag.read_messages(topics=[pc_topic, tf_topic]):
    print t, topic, msg
bag.close()
