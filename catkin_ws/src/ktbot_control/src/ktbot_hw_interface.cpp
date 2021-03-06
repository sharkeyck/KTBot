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
#include <algorithm>

#include <ktbot_control/ktbot_hw_interface.h>

using std::string;
using std::min;
using std::max;

#define LEFT_JOINT_ID 0
#define LEFT_JOINT_SERVO_ID 1

#define RIGHT_JOINT_ID 1
#define RIGHT_JOINT_SERVO_ID 0

#define RUNNING_AVG_NEW_WEIGHT 0.5
#define ROLLOVER_THRESHOLD_DEG 100
#define PI 3.14159265359

// Max based on observed speed of servos
// https://docs.google.com/spreadsheets/d/1uo1gqp0hbpzP6kxLgmkKQCryi_hYWL6A87Cw1xKioBs/edit?usp=sharing
#define MAX_RAD_PER_SEC 41.888
#define MIN_RAD_PER_SEC 15.71


int set_interface_attribs (int fd, int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void set_blocking (int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d setting term attributes", errno);
  }
}



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
  if (set_interface_attribs(ser, B115200, 0) == -1) {
    ROS_FATAL_NAMED("ktbot_hw_interface", "failed to set port settings");
  }
  set_blocking(ser, false);

  left.attach(ser, LEFT_JOINT_SERVO_ID);
  right.attach(ser, RIGHT_JOINT_SERVO_ID);
  left.setCompliance(false);
  right.setCompliance(false);

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

    // ROS_INFO_NAMED("ktbot_hw_interface", "%02f\t%02f\t%02f\t%02f\t%02f\n", deg, d, left_rotations, newpos, joint_velocity_[LEFT_JOINT_ID]);

    // Note: Joint effort not necessary for velocity
    left_prev_deg = deg;
  } else {
    ROS_INFO_NAMED("ktbot_hw_interface", "No data from left wheel\n");
  }
  if (right.getPos(&deg)) {
    double d = deg - right_prev_deg;
    if (abs(d) > ROLLOVER_THRESHOLD_DEG) {
      right_rotations += (d > 0) ? -1 : 1;
    }
    double newpos = ((360.0*right_rotations) + deg) * PI / 180.0;
    double newvel = double(newpos - joint_position_[RIGHT_JOINT_ID]) / elapsed_time.toSec();
    joint_velocity_[RIGHT_JOINT_ID] = (joint_velocity_[RIGHT_JOINT_ID] * (1.0 - RUNNING_AVG_NEW_WEIGHT) - newvel * RUNNING_AVG_NEW_WEIGHT);
    joint_position_[RIGHT_JOINT_ID] = -newpos;

    // OS_INFO_NAMED("ktbot_hw_interface", "%02f\t%02f\t%02f\t%02f\t%02f\n", deg, d, right_rotations, newpos, joint_velocity_[RIGHT_JOINT_ID]);

    // Note: Joint effort not necessary for velocity
    right_prev_deg = deg;
  } else {
    ROS_INFO_NAMED("ktbot_hw_interface", "No data from right wheel\n");
  }

}

// 400RPM max
//

void KTBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  float left_temp_cmd = max(min(joint_velocity_command_[LEFT_JOINT_ID], MAX_RAD_PER_SEC), -MAX_RAD_PER_SEC);
  if (abs(left_temp_cmd) < MIN_RAD_PER_SEC) {
    left_temp_cmd = ((left_temp_cmd < 0) ? -1 : 1) * MIN_RAD_PER_SEC;
  }
  float right_temp_cmd = max(min(joint_velocity_command_[RIGHT_JOINT_ID], MAX_RAD_PER_SEC), -MAX_RAD_PER_SEC);
  if (abs(right_temp_cmd) < MIN_RAD_PER_SEC) {
    right_temp_cmd = ((right_temp_cmd < 0) ? -1 : 1) * MIN_RAD_PER_SEC;
  }

  // https://docs.google.com/spreadsheets/d/1uo1gqp0hbpzP6kxLgmkKQCryi_hYWL6A87Cw1xKioBs/edit#gid=2143003403
  // ROS_INFO_NAMED("ktbot_hw_interface", "%02f\n", joint_velocity_command_[LEFT_JOINT_ID]);
  if (left_prev_vel_cmd != left_temp_cmd) {
    // Convert to calibrated speed values
    int16_t speed = (left_temp_cmd < 0) ? -(-left_temp_cmd + 14.5) / 0.573 : (left_temp_cmd + 8.58)/0.552;
    left.setWheelMode(speed);
    left_prev_vel_cmd = left_temp_cmd;
  }
  if (right_prev_vel_cmd != right_temp_cmd) {
    // Convert to calibrated speed values
    int16_t speed = (right_temp_cmd < 0) ? -(-left_temp_cmd + 12.7) / 0.455 : (left_temp_cmd + 19)/0.511;
    right.setWheelMode(int16_t(speed));
    right_prev_vel_cmd = right_temp_cmd;
  }
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
