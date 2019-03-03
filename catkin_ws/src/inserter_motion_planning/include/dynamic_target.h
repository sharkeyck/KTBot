
class DynamicTarget {

  // Assuming fixed velocity (twist), get the odometry (timestamped pose & twist) most
  // closely matching the pose of the destination
  nav_msgs::Odometry nearest_pose_match(const nav_msgs::Pose dest) {
    // TODO compute vector between odom_ and dest poses

    // TODO project computed vector onto linear velocity from twist

    // TODO divide magnitude of projection by magnitude of linear velocity to get seconds to closest destination

    // TODO return closest destination pose, copied twist, and updated time (based on div of magnitudes).

    return odom_;
  }

  // Compute the expected pose of the target at time T.
  geometry_msgs::Pose pose_at(ros::Time t);
private:
  nav_msgs::Odometry odom_;
}
