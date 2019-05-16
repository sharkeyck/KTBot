import rosparam

class Environment:
  def __init__(self, num_robots=1, positions=None):
    self.num_robots = num_robots
    self.positions = positions or self.gridPositions()

  def gridPositions(self):
    return [] # TODO


def gen_joint_limits(env):
  return ""

def gen_kinematics(env):
  return ""

def gen_fake_controllers(env):
  # controller_list:
  # - name: fake_arm_controller
  #   joints:
  #     - joint1
  #     - joint2
  #     - joint3
  # - name: fake_hand_controller
  #   joints:
  #     - joint5

  return ""

def gen_inserter_srdf(env):
  # xml = subprocess.check_output(['xacro', '--inorder', base_path + 'simulation/inserter/inserter.xacro', 'robot_name:={name}'.format(name=name)])
  # Load the URDF into the ROS Parameter Server - this must be done for each robot
  # in order for the gazebo_ros_control plugin to have a different namespace for each robot.
  # rospy.set_param('/'+name+'/robot_description', xml)

  return ""

def gen_ros_controllers(env):
  return ""

def publishParameters(env):
  <group ns="$(arg robot_description)_planning">
    <rosparam command="load" file="$(find inserter_motion_planning)/config/joint_limits.yaml"/>
  </group>
  <group ns="$(arg robot_description)_kinematics">
    <rosparam command="load" file="$(find inserter_motion_planning)/config/kinematics.yaml"/>
  </group>


  paramlist = rosparam.load_file(base_path + 'config/joints.yaml', default_namespace=name, verbose=True)
  for params, ns in paramlist:
    rosparam.upload_params(ns, params, verbose=True)

    rosparam.set_param("", params, verbose=True)

def main():
  publishParameters(Environment(1, None))

if __name__ == "__main__":
  main()
