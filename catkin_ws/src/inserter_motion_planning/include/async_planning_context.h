

// Inspiration:
// https://github.com/ros-planning/moveit/blob/a1b0efb855af5798d62c4c450e06234abe670bd2/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp#L88:49

class MultiArmPlanningContext : public planning_interface::PlanningContext {
  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  void clear() override;
  bool terminate() override;
}
