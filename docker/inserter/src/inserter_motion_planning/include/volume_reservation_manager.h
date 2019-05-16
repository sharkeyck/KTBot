
// An object shared between multiple robots that allows a robot to reserve a particular
// volume of 3D space. Robots other than the reserver treat this reserved volume as
// an obstacle for the purposes of path planning.
class VolumeReservationManager {

  // Registers a planning scene interface for a particula robot, keyed by the robot's base link frame.
  void registerPlanningSceneInterface(std::string frame_id, const moveit::planning_interface::PlanningSceneInterface::Ptr psi_ptr) {
    scenes.push_back(psi_ptr);
  }

  // Attempts to reserve a volume of 3D space for use by a particular robot.
  // Returns true if no other robot has already acquired a portion of this volume, false otherwise.
  // On success, it releases all other volumes previously held by the robot.
  // Volumes are assocatied with robots via the id (which should be the name of the robot + '_volume')
  bool reserve(moveit_msgs::CollisionObject volume) {
    // TODO may need to mutex here?

    for (auto& scene : scenes) {
      if (hasConflictingCollisionObject(scene, volume)) {
        return false;
      }
    }
    for (auto& scene : scenes) {
      scene->removeCollisionObjects({volume.id});
      scene->addCollisionObjects({volume});
    }
    return true;
  }

  bool hasConflictingCollisionObject() {
    return true; // TODO
  }

private:
  std::vector<moveit::planning_interface::PlanningSceneInterface::Ptr> scenes;
}
