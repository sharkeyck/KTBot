/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the KTBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h> 

#include <ktbot_control/ktbot_hw_interface.h>

using std::string;

#define LEFT_JOINT_ID 0
#define LEFT_JOINT_SERVO_ID 0

#define RIGHT_JOINT_ID 1
#define RIGHT_JOINT_SERVO_ID 1

#define RUNNING_AVG_NEW_WEIGHT 0.05
#define ROLLOVER_THRESHOLD_DEG 100
#define PI 3.14159265359

// @ 7.4v
#define MAX_RPM 3.27249235

namespace ktbot_control
{

KTBotHWInterface::KTBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  // Discover the serial device 
  //const char* path = get_path_for_serial_id("1a86_USB2.0-Serial");
  const char* port = "/dev/ttyUSB0";

  int ser; // file description for the serial port
  ser = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if(ser == -1) {
    ROS_FATAL_NAMED("ktbot_hw_interface", "open_port: Unable to open port %s\n", port);
  } else {
    fcntl(ser, F_SETFL, 0);
    ROS_INFO_NAMED("ktbot_hw_interface", "port opened: %s\n", port);
  }
  
  left.attach(ser, LEFT_JOINT_SERVO_ID);
  right.attach(ser, RIGHT_JOINT_SERVO_ID);

  left_rotations = 0;
  right_rotations = 0;

  left_prev_deg = 0;
  right_prev_deg = 0;

  left_prev_vel_cmd = 0;
  right_prev_vel_cmd = 0;

  ROS_INFO_NAMED("ktbot_hw_interface", "KTBotHWInterface Ready.");
}

void KTBotHWInterface::read(ros::Duration &elapsed_time)
{
  // TODO: Dedupe this code
  float deg;
  if (left.getPos(&deg)) {
    double d = deg - left_prev_deg;
    if (abs(d) > ROLLOVER_THRESHOLD_DEG) {
      left_rotations += (d > 0) ? -1 : 1;
    }
    double newpos = ((360.0*left_rotations) + deg) * PI / 180.0;
    double newvel = double(newpos - joint_position_[LEFT_JOINT_ID]) / elapsed_time.toSec();
    joint_velocity_[LEFT_JOINT_ID] = (joint_velocity_[LEFT_JOINT_ID] * (1.0 - RUNNING_AVG_NEW_WEIGHT) + newvel * RUNNING_AVG_NEW_WEIGHT);
    joint_position_[LEFT_JOINT_ID] = newpos;
    
    // ROS_INFO_NAMED("ktbot_hw_interface", "%02f\t%02f\t%02f\t%02f\n", deg, d, left_rotations, newpos);
    
    // Note: Joint effort not necessary for velocity
    left_prev_deg = deg;
  } else {
    ROS_INFO_NAMED("ktbot_hw_interface", "No data from left wheel\n");
  }
  if (right.getPos(&deg)) {
    double d = -deg - right_prev_deg;
    if (abs(d) > ROLLOVER_THRESHOLD_DEG) {
      right_rotations += (d > 0) ? -1 : 1;
    }
    double newpos = ((360.0*right_rotations) + deg) * PI / 180.0;
    double newvel = double(newpos - joint_position_[RIGHT_JOINT_ID]) / elapsed_time.toSec();
    joint_velocity_[RIGHT_JOINT_ID] = (joint_velocity_[RIGHT_JOINT_ID] * (1.0 - RUNNING_AVG_NEW_WEIGHT) + newvel * RUNNING_AVG_NEW_WEIGHT);
    joint_position_[RIGHT_JOINT_ID] = newpos;
    
    // ROS_INFO_NAMED("ktbot_hw_interface", "%02f\t%02f\t%02f\t%02f\n", deg, d, left_rotations, newpos);
    
    // Note: Joint effort not necessary for velocity
    left_prev_deg = deg;
  } else {
    ROS_INFO_NAMED("ktbot_hw_interface", "No data from right wheel\n");
  }
}

void KTBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Just go by position for now
  // TODO: Rate-limit writes if no change in position command
  // ROS_INFO_NAMED("ktbot_hw_interface", "%02f\n", joint_velocity_command_[LEFT_JOINT_ID]);
  if (left_prev_vel_cmd != joint_velocity_command_[LEFT_JOINT_ID]) {
    left.setWheelMode(int16_t(joint_velocity_command_[LEFT_JOINT_ID] * 1000 / MAX_RPM));
    left_prev_vel_cmd = joint_velocity_command_[LEFT_JOINT_ID];
  }
  if (right_prev_vel_cmd != joint_velocity_command_[RIGHT_JOINT_ID]) {
    right.setWheelMode(int16_t(joint_velocity_command_[RIGHT_JOINT_ID] * -1000 / MAX_RPM));
    right_prev_vel_cmd = joint_velocity_command_[RIGHT_JOINT_ID];
  }

  // TODO joint_velocity_command_
}

void KTBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
