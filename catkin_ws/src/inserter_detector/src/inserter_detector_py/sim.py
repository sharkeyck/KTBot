import roslaunch
import rospy
import rosparam
import argparse
import subprocess

# See https://github.com/ros/ros_comm/tree/kinetic-devel/tools/roslaunch/src/roslaunch
# for source code for roslaunch package.

def spawn_inserter(launch, name, xyz):
  # Spawn a namespaced inserter bot at the xyz position
  base_path = 'src/inserter_detector/'

  # Resolve the xacro file
  xml = subprocess.check_output(['xacro', '--inorder', base_path + 'simulation/inserter/inserter.xacro', 'robot_name:={name}'.format(name=name)])

  # Load the URDF into the ROS Parameter Server - this must be done for each robot
  # in order for the gazebo_ros_control plugin to have a different namespace for each robot.
  rospy.set_param('/'+name+'/robot_description', xml)

  # Spawn the model
  model_args = '-urdf -model {name} -param /{name}/robot_description -x {xyz[0]:.2f} -y {xyz[1]:.2f} -z {xyz[2]:.2f}'.format(name=name, xyz=xyz)
  model_node = roslaunch.core.Node('gazebo_ros', 'spawn_model', namespace=name, respawn=False, output='screen', args=model_args)
  launch.launch(model_node)

  # Load the joints yaml file
  paramlist = rosparam.load_file(base_path + 'config/joints.yaml', default_namespace=name, verbose=True)
  for params, ns in paramlist:
    rosparam.upload_params(ns, params, verbose=True)

  # Start the joint state controller within the namespace
  joint_node = roslaunch.core.Node('controller_manager', 'spawner', namespace=name, args='joint_state_controller arm_controller')
  launch.launch(joint_node)

  # Publish the static transform from the world to the robot
  tf_args = '{xyz[0]:.2f} {xyz[1]:.2f} {xyz[2]:.2f} 0 0 0 world /{name}/base_link 100'.format(name=name, xyz=xyz)
  tf_node = roslaunch.core.Node('tf', 'static_transform_publisher', namespace=name, respawn=False, output='screen', args=tf_args)
  launch.launch(tf_node)

def spawn_prediction(launch):
  detector_node = roslaunch.core.Node(
    'inserter_detector',
    'node',
    namespace='inserter_detector',
    remap_args=[
      ('input', '/realsense/depth_registered/points'),
      ('output', 'cloud_segments'),
    ],
    output='screen',
  )
  launch.launch(detector_node)

  # <!-- For debugging, show performance of the point cloud code -->
  # <node name="perf" pkg="rqt_plot" type="rqt_plot" args="/inserter_detector/inserter0/joint_state_predicted[0]:[1]:[2]"/>
  # rosrun rqt_plot rqt_plot /inserter_detector/inserter0/joint_states_predicted/position[0] /inserter0/joint_states/position[0]

  inference_node = roslaunch.core.Node(
    'inserter_detector',
    'inference_node',
    namespace='inserter_detector',
    remap_args=[
      ('input', 'cloud_segments'),
      ('output', 'joint_predictions'),
    ],
    output='screen',
    args='--model_json_path /home/semartin/Documents/KTBot/catkin_ws/src/inserter_detector/src/models/2018_01_05_00/model.json --weights_path /home/semartin/Documents/KTBot/catkin_ws/src/inserter_detector/src/models/2018_01_05_00/weights.h5')
  launch.launch(inference_node)



def _nearest_square(limit):
    answer = 0
    while answer**2 < limit:
        answer += 1
    return answer

def main():
  parser = argparse.ArgumentParser(description='Sets up a simulation environment with a number of namespaced robots')
  parser.add_argument('--num_bots', type=int, help='number of robots', default=1)
  parser.add_argument('--spacing', type=float, help='spacing, in meters', default=0.5)
  parser.add_argument('--env', type=str, help='parent launch file to use', default='/home/semartin/Documents/KTBot/catkin_ws/src/inserter_detector/launch/gazebo_empty.launch')
  parser.add_argument('--predict', dest='predict', action='store_true', help='predict positions using point cloud')
  args = parser.parse_args()

  rospy.init_node('inserter_sim', anonymous=True)
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.scriptapi.ROSLaunch()
  launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [args.env])
  launch.start()
  rospy.loginfo("Started environment %s" % args.env)

  rospy.sleep(2.0)
  side = _nearest_square(args.num_bots)
  rospy.loginfo("Spawning robots in a {0}x{0} grid".format(side))
  for i in xrange(args.num_bots):
    origin = -args.spacing*(side-1)/2
    row = i // side
    col = i % side
    xyz = (
      origin + args.spacing * col,
      origin + args.spacing * row,
      0,
    )
    name = 'inserter'+str(i)
    rospy.loginfo("Spawning {name} at {xyz}".format(name=name, xyz=xyz))
    spawn_inserter(launch, name, xyz)

  if args.predict:
    spawn_prediction(launch)

  # package = 'rqt_gui'
  # executable = 'rqt_gui'
  # node = roslaunch.core.Node(package, executable)
  # process = launch.launch(node)
  # print process.is_alive()

  # rospy.sleep(3)
  # 3 seconds later
  try:
    launch.spin()
  finally:
    launch.stop()
