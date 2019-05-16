

std::vector<InserterContinuousSolver> solvers;

void targetUpdate(std::vector<nav_msgs::Odometry> targets) {
  for (auto& s : solvers) {
    s.setTargets(targets);
  }
}

int main(int argc, void** argv) {
  // TODO ros setup things

  // TODO Initialize volume reservation manager

  // TODO Load source/dest points for all robots

  // TODO initialize planning groups/scenes (namespaced)

  // TODO subscribe to position updates for targets
}
