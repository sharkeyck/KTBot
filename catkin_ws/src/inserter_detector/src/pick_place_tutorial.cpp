/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define FRAME_ID "base_link"
#define PLANNING_GROUP "arm"

void gripperAction(trajectory_msgs::JointTrajectory& posture, float val) {
  posture.joint_names.resize(1);
  posture.joint_names[0] = "joint5";
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = val;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void openGripper(trajectory_msgs::JointTrajectory& posture) {
  gripperAction(posture, 0.05);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  gripperAction(posture, 0.005);
}

moveit_msgs::CollisionObject genTable(std::string id, float x, float y) {
  moveit_msgs::CollisionObject obj;
  // Add the first table where the cube will originally be kept.
  obj.id = id;
  obj.header.frame_id = FRAME_ID;

  /* Define the primitive and its dimensions. */
  obj.primitives.resize(1);
  obj.primitives[0].type = obj.primitives[0].BOX;
  obj.primitives[0].dimensions.resize(3);
  obj.primitives[0].dimensions[0] = 0.1;
  obj.primitives[0].dimensions[1] = 0.1;
  obj.primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position.x = x;
  obj.primitive_poses[0].position.y = y;
  obj.primitive_poses[0].position.z = -0.03;
  // END_SUB_TUTORIAL

  obj.operation = obj.ADD;
  return obj;
}

moveit_msgs::CollisionObject genObj(std::string id, float x, float y, float z) {
  moveit_msgs::CollisionObject obj;
  // Add the first table where the cube will originally be kept.
  obj.id = id;
  obj.header.frame_id = FRAME_ID;

  /* Define the primitive and its dimensions. */
  obj.primitives.resize(1);
  obj.primitives[0].type = obj.primitives[0].BOX;
  obj.primitives[0].dimensions.resize(3);
  obj.primitives[0].dimensions[0] = 0.04;
  obj.primitives[0].dimensions[1] = 0.04;
  obj.primitives[0].dimensions[2] = 0.04;

  /* Define the pose of the table. */
  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position.x = x;
  obj.primitive_poses[0].position.y = y;
  obj.primitive_poses[0].position.z = z;
  // END_SUB_TUTORIAL

  obj.operation = obj.ADD;
  return obj;
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(genTable("table1", 0.0, -0.2));
  collision_objects.emplace_back(genTable("table2", 0.0,  0.2));

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects.emplace_back(genObj("object", 0, -0.2, 0.04));

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void doMove(moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools, double x, double y, double z) {
  move_group.setPositionTarget(x, y, z, "link_arm_end");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Move plan %s", success ? "SUCCESS" : "FAILED");

  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group.execute(my_plan);
}

void doInserterAction(moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools, int dir) {
  const double yclear = 0.05;
  const double ygrab = 0.167;
  const double zup = 0.1;
  const double zgrab = 0.04;

  doMove(move_group, visual_tools, 0.0, -dir*ygrab, zgrab);

  move_group.attachObject("object");

  doMove(move_group, visual_tools, 0.0, -dir*ygrab, zup);

  doMove(move_group, visual_tools, 0.0, dir*ygrab, zup);

  doMove(move_group, visual_tools, 0.0, dir*ygrab, zgrab);

  move_group.detachObject("object");

  doMove(move_group, visual_tools, 0.0, dir*yclear, zgrab);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "inserter_pick_place_demo");
  ros::NodeHandle nh;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(FRAME_ID);
  visual_tools.deleteAllMarkers();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // move_group.setMaxVelocityScalingFactor(1.0);

  planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

  move_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  while (true) {
    doInserterAction(move_group, visual_tools, 1);
    doInserterAction(move_group, visual_tools, -1);
  }
  /*
  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);
  */

  ros::waitForShutdown();
  return 0;
}
