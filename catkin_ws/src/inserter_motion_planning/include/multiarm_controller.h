

// A controller for multiple inserters.
class MultiInserterController {
  // Ideas:
  // - planning_scene::PlanningScenePtr with a custom CollisionDetector. May need to encode targets into a "robot state"
  // - Use the planning_interface::PlanningContext interface (pointed to e.g. OPML)

  // Set the number of arms present in the scene.
  // All dynamic joints with constraints to be solved must be
  // present in this list to eliminate possible collisions.
  void set_arms(std::vector<InserterArm> arms) {
    for (const auto& arm : arms) {
      arm_map_[arm.name] = arm;
    }
  }

  // Set obstacles the arms must avoid in the scene.
  void set_obstacles(std::vector<Obstacle> obstacles) {

  }

  // Plan and update the trajectories for the arms given a list of targets
  // that have changed since the last planning cycle.
  void async_update_trajectories_for_targets(std::vector<std::string> changed_targets) {
    boost::fiber f(self.solve_from_changes, changed_targets);
  }
}
