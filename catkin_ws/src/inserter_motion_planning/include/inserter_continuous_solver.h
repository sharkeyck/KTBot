
// This class is a continuous solver built for a "warehouse" environment
// where multiple fixed robotic arms are coordinating pick-and-place actions
// on a pipeline of multiple dynamic (moving) targets.
// The solver reserves a volume of space and applies a path constraint to
// stay within the reservation.
// An "inserter arm" wants to pick up objects at a fixed origin and
// place them at a fixed destination.
class InserterContinuousSolver {

  void setPickPoint(geometry_msgs::Point target) {
    src_point_ = target;
  }

  void setPlacePoint(geometry_msgs::Point target) {
    dest_point_ = target;
  }

  void setTargets(std::vector<std::string> changed_targets) {
    MotionPlanRequest mpreq;
    for (const auto& t : changed_targets) {
      if (isPickable(t)) {
        addPickingConstraint(mpreq, t);
      } else {
        // Plan placement trajectory to avoid picking robots
        addPlacingConstraint(mpreq, t);
      }
    }

    // TODO pick the nearest target and plan for it

    setMotionPlanRequest(mpreq);
    // Construct a MotionPlanRequest

    planning_context_->solve(timeout, count);
    simplifySolution(double timeout);

    // https://github.com/ros-planning/moveit/blob/aac8c0de00d5f01c2c3e908b8f4028c84756a920/moveit_core/planning_interface/include/moveit/planning_interface/planning_response.h#L60
    MotionPlanDetailedResponse mpresp;
  }

private:
  std::string name;
  ros::Publisher trajectory_publisher;
  RobotState current_state;
  RobotDescription robot_description;
  tf::Transform base_transform;
  bool gripped_target;
  gemoetry_msgs::Point src_point_;
  gemoetry_msgs::Point dest_point_;

  bool isPickable(const DynamicTarget& t) {
    // t.pose_at(ros::Now()) within R of an arm that doesn't have a grasped target
    return false; // TODO
  }

  // Add a constraint to match the trajectory of the target at the robot's optimal picking point.
  void addPickingConstraint(MotionPlanRequest& mpreq, const DynamicTarget& target) {

  }

  void addPlacingConstraint(MotionPlanRequest& mpreq, const DynamicTarget& target) {

  }

  // Solve the 1D constraint of time for grasping the target when it intercepts
  void adjustTrajectoryForIntercept(MotionPlanDetailedResponse& mpresp, const DynamicTarget& target) {

  }

  std::map<std::string, InserterArm> arm_map_;
  std::map<std::string, DynamicTarget> target_map_;
  planning_interface::PlanningContextPtr planning_context_;
}
