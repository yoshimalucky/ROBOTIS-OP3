/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * thormang3_online_walking.cpp
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */

#include <iostream>
#include <stdio.h>
#include "op3_online_walking_module/op3_online_walking.h"


using namespace robotis_op;


static const double MMtoM = 0.001;
static const double MStoS = 0.001;

static const int NO_STEP_IDX = -1;

static const double TIME_UNIT = 0.008;

static const int IN_WALKING_STARTING = 0;
static const int IN_WALKING = 1;
static const int IN_WALKING_ENDING = 2;

static const int LEFT_FOOT_SWING  = 1;
static const int RIGHT_FOOT_SWING = 2;
static const int STANDING = 3;

static const int BalancingPhase0 = 0; // DSP : START
static const int BalancingPhase1 = 1; // DSP : R--O->L
static const int BalancingPhase2 = 2; // SSP : L_BALANCING1
static const int BalancingPhase3 = 3; // SSP : L_BALANCING2
static const int BalancingPhase4 = 4; // DSP : R--O<-L
static const int BalancingPhase5 = 5; // DSP : R<-O--L
static const int BalancingPhase6 = 6; // SSP : R_BALANCING1
static const int BalancingPhase7 = 7; // SSP : R_BALANCING2
static const int BalancingPhase8 = 8; // DSP : R->O--L
static const int BalancingPhase9 = 9; // DSP : END

static const int StepDataStatus1 = 1; //
static const int StepDataStatus2 = 2; //
static const int StepDataStatus3 = 3; //
static const int StepDataStatus4 = 4; //


OP3OnlineWalking::OP3OnlineWalking()
{
  thormang3_kd_ = new OP3KinematicsDynamics(WholeBody);

  present_right_foot_pose_.x = 0.0;    present_right_foot_pose_.y = -0.5*thormang3_kd_->leg_side_offset_m_;
  present_right_foot_pose_.z = -0.630*MMtoM;
  present_right_foot_pose_.roll = 0.0; present_right_foot_pose_.pitch = 0.0; present_right_foot_pose_.yaw = 0.0;

  present_left_foot_pose_.x = 0.0;    present_left_foot_pose_.y = 0.5*thormang3_kd_->leg_side_offset_m_;
  present_left_foot_pose_.z = -0.630*MMtoM;
  present_left_foot_pose_.roll = 0.0; present_left_foot_pose_.pitch = 0.0; present_left_foot_pose_.yaw = 0.0;

  present_body_pose_.x = 0.0;    present_body_pose_.y = 0.0;     present_body_pose_.z = 0.0;
  present_body_pose_.roll = 0.0; present_body_pose_.pitch = 0.0; present_body_pose_.yaw = 0;

  previous_step_right_foot_pose_  = present_right_foot_pose_;
  previous_step_left_foot_pose_   = present_left_foot_pose_;
  previous_step_body_pose_        = present_body_pose_;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  mat_cob_to_rhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->op3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_rhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->op3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)), 0.0);
  mat_cob_to_lhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->op3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_lhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->op3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0)), 0.0);

  mat_rfoot_to_rft_ = robotis_framework::getRotation4d(M_PI,0,0);
  mat_lfoot_to_lft_ = robotis_framework::getRotation4d(M_PI,0,0);
  rot_x_pi_3d_ = robotis_framework::getRotationX(M_PI);
  rot_z_pi_3d_ = robotis_framework::getRotationZ(M_PI);


  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
      present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_ * mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_ * mat_g_to_lfoot_;

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_rfoot_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_lfoot_);

  thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
  thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);


  r_shoulder_dir_ = thormang3_kd_->op3_link_data_[ID_R_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  r_elbow_dir_    = thormang3_kd_->op3_link_data_[ID_R_ARM_START+2*3]->joint_axis_.coeff(2, 0);
  l_shoulder_dir_ = thormang3_kd_->op3_link_data_[ID_L_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  l_elbow_dir_    = thormang3_kd_->op3_link_data_[ID_L_ARM_START+2*3]->joint_axis_.coeff(2, 0);


  r_init_shoulder_angle_rad_ = r_init_elbow_angle_rad_ = r_shoulder_out_angle_rad_ = r_elbow_out_angle_rad_ = 0;
  l_init_shoulder_angle_rad_ = l_init_elbow_angle_rad_ = l_shoulder_out_angle_rad_ = l_elbow_out_angle_rad_ = 0;

  goal_waist_yaw_angle_rad_ = 0.0*M_PI;

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.1;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.05;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 1.6;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  shouler_swing_gain_    = 0.05;
  elbow_swing_gain_     = 0.1;

  real_running = false; ctrl_running = false;
  current_step_data_status_ = StepDataStatus4;

  walking_time_ = 0; reference_time_ = 0;
  balancing_index_ = BalancingPhase0;

  preview_time_ = 1.6;
  preview_size_ = round(preview_time_/TIME_UNIT);

  //These parameters are for preview control
  control_time_unit_msec_ = 8;
  lipm_height_ = 0.5;
  k_s_ = 0;
  current_start_idx_for_ref_zmp_ = 0;
  ref_zmp_x_at_this_time_ = 0;
  ref_zmp_y_at_this_time_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;

  // variables for balance
  hip_roll_feedforward_angle_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_  = current_left_fy_N_  = current_left_fz_N_  = 0;
  current_left_tx_Nm_ = current_left_ty_Nm_ = current_left_tz_Nm_ = 0;

  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  total_mass_of_robot_ = thormang3_kd_->calcTotalMass(0);

  right_dsp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8*0.5;
  right_ssp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8;
  left_dsp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8*0.5;
  left_ssp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8;

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;


  balance_error_ = robotis_op::BalanceControlError::NoError;
  quat_current_imu_.w() = cos(0.5*M_PI);
  quat_current_imu_.x() = sin(0.5*M_PI);
  quat_current_imu_.y() = 0;
  quat_current_imu_.z() = 0;
}

OP3OnlineWalking::~OP3OnlineWalking()
{  }

bool OP3OnlineWalking::setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
                                            double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
                                            double center_of_body_x, double center_of_body_y, double center_of_body_z,
                                            double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw)
{

  if(real_running || ctrl_running)
    return false;

  previous_step_right_foot_pose_.x     = r_foot_x;
  previous_step_right_foot_pose_.y     = r_foot_y;
  previous_step_right_foot_pose_.z     = r_foot_z;
  previous_step_right_foot_pose_.roll  = r_foot_roll;
  previous_step_right_foot_pose_.pitch = r_foot_pitch;
  previous_step_right_foot_pose_.yaw   = r_foot_yaw;

  previous_step_left_foot_pose_.x     = l_foot_x;
  previous_step_left_foot_pose_.y     = l_foot_y;
  previous_step_left_foot_pose_.z     = l_foot_z;
  previous_step_left_foot_pose_.roll  = l_foot_roll;
  previous_step_left_foot_pose_.pitch = l_foot_pitch;
  previous_step_left_foot_pose_.yaw   = l_foot_yaw;

  previous_step_body_pose_.x     = center_of_body_x;
  previous_step_body_pose_.y     = center_of_body_y;
  previous_step_body_pose_.z     = center_of_body_z;
  previous_step_body_pose_.roll  = center_of_body_roll;
  previous_step_body_pose_.pitch = center_of_body_pitch;
  previous_step_body_pose_.yaw   = center_of_body_yaw;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  return true;
}

void OP3OnlineWalking::setInitalWaistYawAngle(double waist_yaw_angle_rad)
{
  goal_waist_yaw_angle_rad_ = waist_yaw_angle_rad;
}

void OP3OnlineWalking::setInitialRightShoulderAngle(double shoulder_angle_rad)
{
  r_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void OP3OnlineWalking::setInitialLeftShoulderAngle(double shoulder_angle_rad)
{
  l_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void OP3OnlineWalking::setInitialRightElbowAngle(double elbow_angle_rad)
{
  r_init_elbow_angle_rad_ = elbow_angle_rad;
}

void OP3OnlineWalking::setInitialLeftElbowAngle(double elbow_angle_rad)
{
  l_init_elbow_angle_rad_ = elbow_angle_rad;
}

void OP3OnlineWalking::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);

  mat_current_imu_ = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;

  current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));

  imu_data_mutex_lock_.unlock();
}

void OP3OnlineWalking::setPreviewTime(double preview_time)
{
  preview_time_ = preview_time;
}

void OP3OnlineWalking::setPreviewRefHeight(double height)
{
  lipm_height_ = height;
}

void OP3OnlineWalking::initialize(int control_time_unit_msec)
{
  if(real_running)
    return;

  control_time_unit_msec_ = control_time_unit_msec;

  step_data_mutex_lock_.lock();
  added_step_data_.clear();

  // initialize balance
  balance_ctrl_.initialize(TIME_UNIT*1000.0);
  balance_ctrl_.setGyroBalanceEnable(true);
  balance_ctrl_.setOrientationBalanceEnable(false);
  balance_ctrl_.setForceTorqueBalanceEnable(false);

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(previous_step_body_pose_.roll, previous_step_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);


  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
    curr_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    curr_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  for(int feed_forward_idx = 0; feed_forward_idx < 12; feed_forward_idx++)
  {
    leg_angle_feed_back_[feed_forward_idx].p_gain_ = 0;
    leg_angle_feed_back_[feed_forward_idx].d_gain_ = 0;
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;

  // Initialize Matrix for Preview Control
  initializePreviewMatrices();

  step_idx_data_.resize(preview_size_);
  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.resize(preview_size_, 1);
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.resize(preview_size_, 1);
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));

  u_x.resize(1,1);
  u_y.resize(1,1);

  x_lipm_.resize(3, 1);    y_lipm_.resize(3, 1);
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);

  step_data_mutex_lock_.unlock();

  r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
  l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
  r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
  l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;

}

void OP3OnlineWalking::reInitialize()
{
  if(real_running)
    return;

  step_data_mutex_lock_.lock();
  added_step_data_.clear();

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  previous_step_right_foot_pose_ = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_rfoot_);
  previous_step_left_foot_pose_  = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_lfoot_);
  previous_step_body_pose_       = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_cob_);

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);
  mat_g_to_robot_ = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_rhip_to_cob_*mat_cob_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_lhip_to_cob_*mat_cob_to_g_*mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;
  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));
  sum_of_zmp_x_ = 0.0;
  sum_of_zmp_y_ = 0.0;

  sum_of_cx_ = 0.0;
  sum_of_cy_ = 0.0;
  x_lipm_.fill(0.0);        y_lipm_.fill(0.0);

  step_data_mutex_lock_.unlock();

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;
}

void OP3OnlineWalking::initializePreviewMatrices()
{
  // Initialize Matrix for Preview Control
  double t = 0;
  if(control_time_unit_msec_ < 1.0)
    t=control_time_unit_msec_;
  else
    t=control_time_unit_msec_/1000.0;

  preview_size_ = round(preview_time_/t);

  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t, t*t/2.0,
      0,  1,   t,
      0,  0,   1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -200.0*0.001/GRAVITY_ACCELERATION;

  k_s_ = 705.639670950639; //200


  k_x_.resize(1,3);
  k_x_ << 28095.0026650399, 4583.96107321767, 80.7952461119128; // 200


  f_.resize(1, preview_size_);
  //200
  f_ << 705.639671,   786.8914416,    893.8644725,   992.8713032,    1066.696109,   1109.471893,   1122.363772,   1110.261862,    1079.464719,    1036.213524,    985.888593,     932.6774449,    879.5472804,    828.3895748,    780.2410883,   735.5183257,   694.2288262,   656.1420301,   620.9154448,   588.1795754,
        557.5889195,  528.8474986,    501.7169383,   476.0138153,     451.6013893,   428.3792885,   406.2733928,   385.2271309,    365.1946829,    346.1361096,   328.0141725,    310.7924915,    294.4346721,    278.9040709,    264.1639351,   250.1777197,   236.9094511,   224.3240601,   212.3876442,   201.0676482,
        190.3329673,  180.1539852,    170.5025642,   161.3520015,     152.6769664,   144.453427,    136.6585754,   129.2707546,    122.26939,      115.6349271,   109.3487752,    103.3932558,     97.75155627,    92.40768717,    87.34644288,   82.55336454,   78.01470522,   73.71739677,   69.64901812,   65.79776505,
        62.15242104,   58.70232948,    55.43736691,   52.34791746,    49.42484839,   46.65948673,   44.04359692,   41.56935954,    39.22935101,    37.01652421,    34.92419003,   32.94599985,    31.07592874,     29.30825952,    27.63756763,   26.05870658,   24.56679426,   23.15719974,   21.82553084,   20.56762221,
        19.379524,     18.25749109,    17.19797284,   16.19760331,    15.25319197,   14.36171485,   13.52030616,   12.72625024,    11.97697399,    11.27003961,    10.6031377,     9.974080773,    9.38079695,      8.821324102,    8.293804203,   7.796477987,   7.327679868,   6.885833108,   6.469445236,   6.077103681,
         5.707471638,   5.359284134,    5.031344287,   4.722519764,    4.431739408,    4.157990042,   3.90031343,    3.657803393,    3.429603075,    3.214902342,    3.012935322,   2.822978057,    2.644346294,     2.476393369,    2.318508216,   2.170113471,   2.030663671,   1.899643554,   1.776566438,   1.660972692,
         1.552428283,   1.450523394,    1.354871123,   1.265106244,    1.180884032,    1.101879153,   1.027784611,   0.958310752,    0.893184313,    0.832147532,    0.774957299,   0.721384349,    0.671212505,     0.62423796,     0.580268587,   0.539123302,   0.500631448,   0.464632218,   0.430974108 ,  0.399514399,
         0.370118667,   0.342660318,    0.317020151,   0.293085943,    0.270752055,    0.249919064,   0.230493408,   0.212387058,    0.195517201,    0.179805948,    0.165180048,   0.151570629,    0.138912942,     0.127146129,    0.116212998,   0.106059812,   0.096636089,   0.087894414,   0.079790262,   0.07228183,
         0.065329877,   0.058897575,    0.052950371,   0.047455848,    0.042383604,    0.037705129,   0.033393698,   0.029424263,    0.025773351,    0.022418974,    0.019340537,   0.01651876,     0.01393559,      0.011574137,    0.009418596,   0.007454187,   0.005667089,   0.004044382,   0.002573994,   0.001244646,
         4.58E-05,     -0.001032352,   -0.001999011,  -0.002862726,   -0.003631481,   -0.004312715,  -0.004913362,  -0.00543988,    -0.00589828,    -0.006294157,   -0.006632716,  -0.006918794,   -0.007156889,  -0.007351176,   -0.007505532,   -0.007623553,  -0.007708573,  -0.007763683,   -0.007791745,  -0.007795405;


//  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4,4);
//  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4,1);
//  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1,4);
//
//  tempA.coeffRef(0,0) = 1;
//  tempA.block<1,3>(0,1) = c_*A_;
//  tempA.block<3,3>(1,1) = A_;
//
//  tempb.coeffRef(0,0) = (c_*b_).coeff(0,0);
//  tempb.block<3,1>(1,0) = b_;
//
//  tempc.coeffRef(0,0) = 1;
//
//  double Q_e = 1, R = 1e-6;//1.0e-6;
//  double Q_x = 0;
//  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);
//
//  Q.coeffRef(0,0) = Q_e;
//  Q.coeffRef(1,1) = Q_e;
//  Q.coeffRef(2,2) = Q_e;
//  Q.coeffRef(3,3) = Q_x;
//
//  double matrix_A[] = {tempA.coeff(0,0), tempA.coeff(1,0), tempA.coeff(2,0), tempA.coeff(3,0),
//      tempA.coeff(0,1), tempA.coeff(1,1), tempA.coeff(2,1), tempA.coeff(3,1),
//      tempA.coeff(0,2), tempA.coeff(1,2), tempA.coeff(2,2), tempA.coeff(3,2),
//      tempA.coeff(0,3), tempA.coeff(1,3), tempA.coeff(2,3), tempA.coeff(3,3)};
//  int row_A = 4, col_A = 4;
//
//  double matrix_B[] = {tempb.coeff(0,0), tempb.coeff(1,0), tempb.coeff(2,0), tempb.coeff(3,0)};
//  int row_B = 4, col_B = 1;
//
//  double matrix_Q[] = {Q.coeff(0,0), Q.coeff(1,0), Q.coeff(2,0), Q.coeff(3,0),
//      Q.coeff(0,1), Q.coeff(1,1), Q.coeff(2,1), Q.coeff(3,1),
//      Q.coeff(0,2), Q.coeff(1,2), Q.coeff(2,2), Q.coeff(3,2),
//      Q.coeff(0,3), Q.coeff(1,3), Q.coeff(2,3), Q.coeff(3,3)};
//  int row_Q = 4, col_Q = 4;
//
//  double matrix_R[] = {R};
//  int row_R = 1, col_R = 1;
//
//  double *matrix_K = (double*)malloc(100*sizeof(double));
//  int row_K, col_K;
//
//  double *matrix_P = (double*)malloc(100*sizeof(double));
//  int row_P, col_P;
//
//  double *matrix_E_real = (double*)malloc(100*sizeof(double));
//  double *matrix_E_imag = (double*)malloc(100*sizeof(double));
//  int row_E, col_E;
//
//  robotis_framework::ScilabOptimization::initialize();
//  robotis_framework::ScilabOptimization::solveRiccatiEquation(matrix_K, &row_K, &col_K,
//      matrix_P, &row_P, &col_P,
//      matrix_E_real, matrix_E_imag, &row_E, &col_E,
//      matrix_A, row_A, col_A,
//      matrix_B, row_B, col_B,
//      matrix_Q, row_Q, col_Q,
//      matrix_R, row_R, col_R);
//
//  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(row_K,col_K);
//  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(row_P,col_P);
//  Eigen::MatrixXd E_real = Eigen::MatrixXd::Zero(row_E,col_E);
//  Eigen::MatrixXd E_imag = Eigen::MatrixXd::Zero(row_E,col_E);
//
//  for(int j=0 ; j<row_K ; j++)
//  {
//    for(int i=0 ; i<col_K ; i++)
//      K.coeffRef(j,i) = matrix_K[i*row_K+j];
//  }
//
//  for(int j=0; j<row_P; j++)
//  {
//    for(int i=0 ; i<col_P; i++)
//      P.coeffRef(j,i) = matrix_P[i*row_P+j];
//  }
//
//  for(int j=0 ; j<row_E; j++)
//  {
//    for(int i=0 ; i<col_E ; i++)
//    {
//      E_real.coeffRef(j,i) = matrix_E_real[i*row_E+j];
//      E_imag.coeffRef(j,i) = matrix_E_imag[i*row_E+j];
//    }
//  }
//
//  k_s_ = K.coeff(0,0);
//  k_x_.resize(1,3);
//  k_x_ << K.coeff(0,1), K.coeff(0,2), K.coeff(0,3);
//
//  f_.resize(1, preview_size_);
//
//  Eigen::MatrixXd mat_R = Eigen::MatrixXd::Zero(1,1);
//  mat_R.coeffRef(0,0) = R;
//
//  Eigen::MatrixXd tempCoeff1 = mat_R + ((tempb.transpose() * P) * tempb);
//  tempCoeff1 = tempCoeff1.inverse();
//  Eigen::MatrixXd tempCoeff2 = tempb.transpose();
//  Eigen::MatrixXd tempCoeff3 = Eigen::MatrixXd::Identity(4,4);
//  Eigen::MatrixXd tempCoeff4 = P*tempc.transpose();
//
//  f_.block<1,1>(0,0) = ((tempCoeff1*tempCoeff2)* tempCoeff3) * tempCoeff4;
//
//  for(int i = 1; i < preview_size_; i++)
//  {
//    tempCoeff3 = tempCoeff3*((tempA - tempb*K).transpose());
//    f_.block<1,1>(0,i) = ((tempCoeff1*tempCoeff2)* tempCoeff3) * tempCoeff4;
//  }
//
//  free(matrix_K);
//  free(matrix_P);
//  free(matrix_E_real);
//  free(matrix_E_imag);
//
  u_x.resize(1,1);  u_y.resize(1,1);
  u_x.fill(0.0);    u_y.fill(0.0);

  x_lipm_.resize(3, 1);    y_lipm_.resize(3, 1);
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}

void OP3OnlineWalking::start()
{
  ctrl_running = true;
  real_running = true;
}

void OP3OnlineWalking::stop()
{
  ctrl_running = false;
}

bool OP3OnlineWalking::isRunning()
{
  return real_running;
}

bool OP3OnlineWalking::addStepData(robotis_framework::StepData step_data)
{
  step_data_mutex_lock_.lock();
  added_step_data_.push_back(step_data);

  calcStepIdxData();
  step_data_mutex_lock_.unlock();

  return true;
}

int OP3OnlineWalking::getNumofRemainingUnreservedStepData()
{
  int step_idx = step_idx_data_(preview_size_ - 1);
  int remain_step_num = 0;
  if(step_idx != NO_STEP_IDX)
  {
    remain_step_num = (added_step_data_.size() - 1 - step_idx);
  }
  else
  {
    remain_step_num = 0;
  }
  return remain_step_num;
}

void OP3OnlineWalking::eraseLastStepData()
{
  step_data_mutex_lock_.lock();
  if(getNumofRemainingUnreservedStepData() != 0)
  {
    added_step_data_.pop_back();
  }
  step_data_mutex_lock_.unlock();
}

void OP3OnlineWalking::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;
  (*ref_step_data_for_addition) = reference_step_data_for_addition_;
}

void OP3OnlineWalking::calcStepIdxData()
{
  unsigned int step_idx = 0, previous_step_idx = 0;
  unsigned int step_data_size = added_step_data_.size();
  if(added_step_data_.size() == 0)
  {
    step_idx_data_.fill(NO_STEP_IDX);
    current_step_data_status_ = StepDataStatus4;
    real_running = false;
  }
  else
  {
    if(walking_time_ >= added_step_data_[0].time_data.abs_step_time - 0.5*MStoS)
    {
      previous_step_waist_yaw_angle_rad_ = added_step_data_[0].position_data.waist_yaw_angle;
      previous_step_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      previous_step_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
      previous_step_body_pose_ = added_step_data_[0].position_data.body_pose;
      previous_step_body_pose_.x = present_body_pose_.x;
      previous_step_body_pose_.y = present_body_pose_.y;
      reference_time_ = added_step_data_[0].time_data.abs_step_time;
      added_step_data_.erase(added_step_data_.begin());
      if(added_step_data_.size() == 0)
      {
        step_idx_data_.fill(NO_STEP_IDX);
        current_step_data_status_ = StepDataStatus4;
        real_running = false;
      }
      else
      {
        for(int idx = 0; idx < preview_size_; idx++)
        {
          //Get STepIDx
          if(walking_time_ + (idx+1)*control_time_unit_msec_*0.001 > added_step_data_[step_data_size -1].time_data.abs_step_time)
            step_idx_data_(idx) = NO_STEP_IDX;
          else
          {
            for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
            {
              if(walking_time_ + (idx+1)*control_time_unit_msec_*0.001 <= added_step_data_[step_idx].time_data.abs_step_time)
                break;
            }
            step_idx_data_(idx) = step_idx;
            previous_step_idx = step_idx;
          }
        }
      }
    }
    else
    {
      for(int idx = 0; idx < preview_size_; idx++)
      {
        //Get StepIdx
        if(walking_time_ + (idx+1)*control_time_unit_msec_*0.001 > added_step_data_[step_data_size -1].time_data.abs_step_time)
          step_idx_data_(idx) = NO_STEP_IDX;
        else
        {
          for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
          {
            if(walking_time_ + (idx+1)*control_time_unit_msec_*0.001 <= added_step_data_[step_idx].time_data.abs_step_time)
              break;
          }
          step_idx_data_(idx) = step_idx;
          previous_step_idx = step_idx;
        }
      }
    }
  }

  if(step_idx_data_(preview_size_ - 1) != NO_STEP_IDX)
  {
    if(getNumofRemainingUnreservedStepData() != 0)
    {
      current_step_data_status_ = StepDataStatus1;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
    else
    {
      current_step_data_status_ = StepDataStatus2;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
  }
  else
  {
    if(step_idx_data_(0) != NO_STEP_IDX)
    {
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(0)];
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time += preview_time_;

      current_step_data_status_ = StepDataStatus3;
    }
    else
    {
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time = walking_time_ + preview_time_;
      current_step_data_status_ = StepDataStatus4;
    }
  }
}

void OP3OnlineWalking::calcRefZMP()
{
  int ref_zmp_idx = 0;
  int step_idx = 0;
  if(walking_time_ == 0)
  {
    if((step_idx_data_(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
    {
      reference_zmp_x_.fill((present_left_foot_pose_.x + present_right_foot_pose_.x)*0.5);
      reference_zmp_y_.fill((present_left_foot_pose_.y + present_right_foot_pose_.y)*0.5);
      return;
    }

    for(ref_zmp_idx = 0; ref_zmp_idx < preview_size_;  ref_zmp_idx++)
    {
      step_idx = step_idx_data_(ref_zmp_idx);
      if(step_idx == NO_STEP_IDX)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = reference_zmp_x_(ref_zmp_idx - 1, 0);
        reference_zmp_y_(ref_zmp_idx, 0) = reference_zmp_y_(ref_zmp_idx - 1, 0);
      }
      else
      {
        if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
        {
          if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
          else
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
    }
    current_start_idx_for_ref_zmp_ = 0;
  }
  else
  {
    step_idx = step_idx_data_(preview_size_ - 1);

    if(current_start_idx_for_ref_zmp_ == 0)
      ref_zmp_idx = preview_size_ - 1;
    else
      ref_zmp_idx = current_start_idx_for_ref_zmp_ - 1;

    if(step_idx == NO_STEP_IDX)
    {
      reference_zmp_x_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.x + reference_step_data_for_addition_.position_data.left_foot_pose.x);
      reference_zmp_y_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.y + reference_step_data_for_addition_.position_data.left_foot_pose.y);
    }
    else
    {
      if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
      {
        if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
    }
  }
}

void OP3OnlineWalking::calcDesiredPose()
{
  ////Original LIPM
  //u_x = -K*x_LIPM + x_feed_forward_term;
  //x_LIPM = A*x_LIPM + b*u_x;
  //
  //u_y = -K*y_LIPM + y_feed_forward_term;
  //y_LIPM = A*y_LIPM + b*u_y;

  //Calc LIPM with Integral
  Eigen::MatrixXd  x_feed_forward_term2; //f_Preview*m_ZMP_Reference_X;
  Eigen::MatrixXd  y_feed_forward_term2; //f_Preview*m_ZMP_Reference_Y;

  x_feed_forward_term2.resize(1,1);
  x_feed_forward_term2.fill(0.0);
  y_feed_forward_term2.resize(1,1);
  y_feed_forward_term2.fill(0.0);

  for(int i = 0; i < preview_size_; i++)
  {
    if(current_start_idx_for_ref_zmp_ + i < preview_size_) {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i, 0);
    }
    else {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
    }
  }

  sum_of_cx_ += c_(0,0)*x_lipm_(0,0) +  c_(0,1)*x_lipm_(1,0) +  c_(0,2)*x_lipm_(2,0);
  sum_of_cy_ += c_(0,0)*y_lipm_(0,0) +  c_(0,1)*y_lipm_(1,0) +  c_(0,2)*y_lipm_(2,0);

  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + x_feed_forward_term2(0,0);
  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + y_feed_forward_term2(0,0);
  x_lipm_ = A_*x_lipm_ + b_*u_x;
  y_lipm_ = A_*y_lipm_ + b_*u_y;


  ref_zmp_x_at_this_time_ = reference_zmp_x_.coeff(current_start_idx_for_ref_zmp_, 0);
  ref_zmp_y_at_this_time_ = reference_zmp_y_.coeff(current_start_idx_for_ref_zmp_, 0);

  sum_of_zmp_x_ += reference_zmp_x_.coeff(current_start_idx_for_ref_zmp_, 0);
  sum_of_zmp_y_ += reference_zmp_y_.coeff(current_start_idx_for_ref_zmp_, 0);

  present_body_pose_.x = x_lipm_.coeff(0,0);
  present_body_pose_.y = y_lipm_.coeff(0,0);

  reference_step_data_for_addition_.position_data.body_pose.x = x_lipm_(0,0);
  reference_step_data_for_addition_.position_data.body_pose.y = y_lipm_(0,0);

  current_start_idx_for_ref_zmp_++;
  if(current_start_idx_for_ref_zmp_ == (preview_size_))
    current_start_idx_for_ref_zmp_ = 0;
}

void OP3OnlineWalking::process()
{
  if(!ctrl_running)
  {
    return;
  }
  else
  {
    step_data_mutex_lock_.lock();

    calcStepIdxData();
    calcRefZMP();
    calcDesiredPose();

    double hip_roll_swap = 0;

    if((added_step_data_.size() != 0) && real_running)
    {
      double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, ssp_time_start, ssp_time_end;
      period_time = added_step_data_[0].time_data.abs_step_time - reference_time_;
      dsp_ratio = added_step_data_[0].time_data.dsp_ratio;
      ssp_ratio = 1 - dsp_ratio;
      foot_move_period_time = ssp_ratio*period_time;

      ssp_time_start = dsp_ratio*period_time/2.0 + reference_time_;
      ssp_time_end = (1 + ssp_ratio)*period_time / 2.0 + reference_time_;

      double start_time_delay_ratio_x        = added_step_data_[0].time_data.start_time_delay_ratio_x;
      double start_time_delay_ratio_y        = added_step_data_[0].time_data.start_time_delay_ratio_y;
      double start_time_delay_ratio_z        = added_step_data_[0].time_data.start_time_delay_ratio_z;
      double start_time_delay_ratio_roll     = added_step_data_[0].time_data.start_time_delay_ratio_roll;
      double start_time_delay_ratio_pitch    = added_step_data_[0].time_data.start_time_delay_ratio_pitch;
      double start_time_delay_ratio_yaw      = added_step_data_[0].time_data.start_time_delay_ratio_yaw;
      double finish_time_advance_ratio_x     = added_step_data_[0].time_data.finish_time_advance_ratio_x;
      double finish_time_advance_ratio_y     = added_step_data_[0].time_data.finish_time_advance_ratio_y;
      double finish_time_advance_ratio_z     = added_step_data_[0].time_data.finish_time_advance_ratio_z;
      double finish_time_advance_ratio_roll  = added_step_data_[0].time_data.finish_time_advance_ratio_roll;
      double finish_time_advance_ratio_pitch = added_step_data_[0].time_data.finish_time_advance_ratio_pitch;
      double finish_time_advance_ratio_yaw   = added_step_data_[0].time_data.finish_time_advance_ratio_yaw;

      double hip_roll_swap_dir = 1.0;

      if( (walking_time_ - reference_time_) < control_time_unit_msec_*0.001)
      {
        waist_yaw_tra_.changeTrajectory(reference_time_, previous_step_waist_yaw_angle_rad_, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.waist_yaw_angle, 0, 0);
        body_z_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.z, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.z, 0, 0);
        body_roll_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.roll, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.roll, 0, 0);
        body_pitch_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.pitch, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.pitch, 0, 0);

        double bc_move_amp = added_step_data_[0].position_data.body_pose.yaw - previous_step_body_pose_.yaw;

        if(bc_move_amp >= M_PI)
          bc_move_amp -= 2.0*M_PI;
        else if(bc_move_amp <= -M_PI)
          bc_move_amp += 2.0*M_PI;

        body_yaw_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.yaw, 0, 0,
            added_step_data_[0].time_data.abs_step_time, bc_move_amp + previous_step_body_pose_.yaw, 0, 0);

        body_z_swap_tra_.changeTrajectory(reference_time_,
            0, 0, 0,
            0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
            added_step_data_[0].position_data.body_z_swap, 0, 0);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          foot_x_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_x*foot_move_period_time,
              previous_step_right_foot_pose_.x, 0, 0,
              ssp_time_end - finish_time_advance_ratio_x*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.x, 0, 0);
          foot_y_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_y*foot_move_period_time,
              previous_step_right_foot_pose_.y, 0, 0,
              ssp_time_end - finish_time_advance_ratio_y*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.y, 0, 0);
          foot_z_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_z*foot_move_period_time,
              previous_step_right_foot_pose_.z, 0, 0,
              ssp_time_end - finish_time_advance_ratio_z*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.z, 0, 0);
          foot_roll_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_roll*foot_move_period_time,
              previous_step_right_foot_pose_.roll, 0, 0,
              ssp_time_end - finish_time_advance_ratio_roll*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.roll, 0, 0);
          foot_pitch_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_pitch*foot_move_period_time,
              previous_step_right_foot_pose_.pitch, 0, 0,
              ssp_time_end - finish_time_advance_ratio_pitch*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.pitch, 0, 0);

          double c_move_amp = added_step_data_[0].position_data.right_foot_pose.yaw - previous_step_right_foot_pose_.yaw;
          if(c_move_amp >= M_PI)
            c_move_amp -= 2.0*M_PI;
          else if(c_move_amp <= -M_PI)
            c_move_amp += 2.0*M_PI;

          foot_yaw_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_yaw*foot_move_period_time,
              previous_step_right_foot_pose_.yaw, 0, 0,
              ssp_time_end - finish_time_advance_ratio_yaw*foot_move_period_time,
              c_move_amp + previous_step_right_foot_pose_.yaw, 0, 0);

          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), hip_roll_feedforward_angle_rad_, 0, 0);
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        {
          foot_x_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_x*foot_move_period_time,
              previous_step_left_foot_pose_.x, 0, 0,
              ssp_time_end - finish_time_advance_ratio_x*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.x, 0, 0);
          foot_y_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_y*foot_move_period_time,
              previous_step_left_foot_pose_.y, 0, 0,
              ssp_time_end - finish_time_advance_ratio_y*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.y, 0, 0);
          foot_z_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_z*foot_move_period_time,
              previous_step_left_foot_pose_.z, 0, 0,
              ssp_time_end - finish_time_advance_ratio_z*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.z, 0, 0);
          foot_roll_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_roll*foot_move_period_time,
              previous_step_left_foot_pose_.roll, 0, 0,
              ssp_time_end - finish_time_advance_ratio_roll*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.roll, 0, 0);
          foot_pitch_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_pitch*foot_move_period_time,
              previous_step_left_foot_pose_.pitch, 0, 0,
              ssp_time_end - finish_time_advance_ratio_pitch*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.pitch, 0, 0);

          double c_move_amp = added_step_data_[0].position_data.left_foot_pose.yaw - previous_step_left_foot_pose_.yaw;
          if(c_move_amp >= M_PI)
            c_move_amp -= 2.0*M_PI;
          else if(c_move_amp <= -M_PI)
            c_move_amp += 2.0*M_PI;

          foot_yaw_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_yaw*foot_move_period_time,
              previous_step_left_foot_pose_.yaw, 0, 0,
              ssp_time_end - finish_time_advance_ratio_yaw*foot_move_period_time,
              c_move_amp + previous_step_left_foot_pose_.yaw, 0, 0);

          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), hip_roll_feedforward_angle_rad_, 0, 0);

          hip_roll_swap_dir = 1.0;
        }
        else
        {
          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              ssp_time_end, 0, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              ssp_time_end, 0, 0, 0);

          hip_roll_swap_dir = 0.0;
        }
      }

      double z_swap = body_z_swap_tra_.getPosition(walking_time_);
      double wp_move = waist_yaw_tra_.getPosition(walking_time_);
      double bz_move = body_z_tra_.getPosition(walking_time_);
      double ba_move = body_roll_tra_.getPosition(walking_time_);
      double bb_move = body_pitch_tra_.getPosition(walking_time_);
      double bc_move = body_yaw_tra_.getPosition(walking_time_);

      present_waist_yaw_angle_rad_ = wp_move;
      present_body_pose_.z = bz_move + z_swap;
      present_body_pose_.roll = ba_move;
      present_body_pose_.pitch = bb_move;
      present_body_pose_.yaw = bc_move;

      //Feet
      double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;
      if( walking_time_ <= ssp_time_start)
      {
        x_move = foot_x_tra_.getPosition(ssp_time_start);
        y_move = foot_y_tra_.getPosition(ssp_time_start);
        z_move = foot_z_tra_.getPosition(ssp_time_start);
        a_move = foot_roll_tra_.getPosition(ssp_time_start);
        b_move = foot_pitch_tra_.getPosition(ssp_time_start);
        c_move = foot_yaw_tra_.getPosition(ssp_time_start);

        z_vibe = foot_z_swap_tra_.getPosition(ssp_time_start);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(ssp_time_start);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
          balancing_index_ = BalancingPhase1;
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
          balancing_index_ = BalancingPhase5;
        else
          balancing_index_ = BalancingPhase0;
      }
      else if( walking_time_ <= ssp_time_end)
      {
        x_move = foot_x_tra_.getPosition(walking_time_);
        y_move = foot_y_tra_.getPosition(walking_time_);
        z_move = foot_z_tra_.getPosition(walking_time_);
        a_move = foot_roll_tra_.getPosition(walking_time_);
        b_move = foot_pitch_tra_.getPosition(walking_time_);
        c_move = foot_yaw_tra_.getPosition(walking_time_);

        if(added_step_data_[0].position_data.moving_foot != STANDING)
        {
          if((walking_time_ >= 0.5*(ssp_time_start + ssp_time_end))
              && (walking_time_ < (0.5*(ssp_time_start + ssp_time_end) + TIME_UNIT)))
          {
            body_z_swap_tra_.changeTrajectory(0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
                added_step_data_[0].position_data.body_z_swap, 0, 0,
                added_step_data_[0].time_data.abs_step_time,
                0, 0, 0);

            foot_z_swap_tra_.changeTrajectory(0.5*(ssp_time_start + ssp_time_end),
                added_step_data_[0].position_data.foot_z_swap, 0, 0,
                ssp_time_end, 0, 0, 0);

            hip_roll_swap_tra_.changeTrajectory(0.5*(ssp_time_start + ssp_time_end),
                hip_roll_feedforward_angle_rad_, 0, 0,
                ssp_time_end, 0, 0, 0);
          }
        }

        z_vibe = foot_z_swap_tra_.getPosition(walking_time_);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(walking_time_);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = BalancingPhase2;
          else
            balancing_index_ = BalancingPhase3;
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        {
          if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = BalancingPhase6;
          else
            balancing_index_ = BalancingPhase7;
        }
        else
          balancing_index_ = BalancingPhase0;
      }
      else
      {
        x_move = foot_x_tra_.getPosition(ssp_time_end);
        y_move = foot_y_tra_.getPosition(ssp_time_end);
        z_move = foot_z_tra_.getPosition(ssp_time_end);
        a_move = foot_roll_tra_.getPosition(ssp_time_end);
        b_move = foot_pitch_tra_.getPosition(ssp_time_end);
        c_move = foot_yaw_tra_.getPosition(ssp_time_end);

        z_vibe = foot_z_swap_tra_.getPosition(ssp_time_end);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(ssp_time_end);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
          balancing_index_ = BalancingPhase4;
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
          balancing_index_ = BalancingPhase8;
        else
          balancing_index_ = BalancingPhase0;
      }


      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
      {
        present_right_foot_pose_.x = x_move;
        present_right_foot_pose_.y = y_move;
        present_right_foot_pose_.z = z_move + z_vibe;
        present_right_foot_pose_.roll = a_move;
        present_right_foot_pose_.pitch = b_move;
        present_right_foot_pose_.yaw = c_move;

        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;

        hip_roll_swap_dir = -1.0;
      }
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
      {
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;

        present_left_foot_pose_.x = x_move;
        present_left_foot_pose_.y = y_move;
        present_left_foot_pose_.z = z_move + z_vibe;
        present_left_foot_pose_.roll = a_move;
        present_left_foot_pose_.pitch = b_move;
        present_left_foot_pose_.yaw = c_move;

        hip_roll_swap_dir = 1.0;
      }
      else
      {
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;

        hip_roll_swap_dir = 0.0;
      }

      hip_roll_swap = hip_roll_swap_dir * hip_roll_swap;

      shouler_swing_gain_ = added_step_data_[0].position_data.shoulder_swing_gain;
      elbow_swing_gain_ = added_step_data_[0].position_data.elbow_swing_gain;

      walking_time_ += control_time_unit_msec_*0.001;

      if(walking_time_ > added_step_data_[added_step_data_.size() - 1].time_data.abs_step_time - 0.5*0.001)
      {
        real_running = false;
        calcStepIdxData();
        step_data_mutex_lock_.unlock();
        reInitialize();
        step_data_mutex_lock_.lock();
      }

      if(balancing_index_ == BalancingPhase0 || balancing_index_ == BalancingPhase9)
      {
        left_fz_trajectory_start_time_ = walking_time_;
        left_fz_trajectory_end_time_   = walking_time_;
        left_fz_trajectory_target_  = left_dsp_fz_N_;
        left_fz_trajectory_shift_   = left_dsp_fz_N_;
      }
      else if(balancing_index_ == BalancingPhase1 )
      {
        left_fz_trajectory_end_time_ = ssp_time_start;
        left_fz_trajectory_target_ = left_ssp_fz_N_;
      }
      else if(balancing_index_ == BalancingPhase4 )
      {
        left_fz_trajectory_start_time_ = ssp_time_end;
        left_fz_trajectory_shift_ = left_ssp_fz_N_;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_trajectory_target_ = left_dsp_fz_N_;
            left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_trajectory_target_ = 0.0;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_trajectory_target_ = left_ssp_fz_N_;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else {
          left_fz_trajectory_target_ = left_dsp_fz_N_;
          left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }
      }
      else if(balancing_index_ == BalancingPhase5 )
      {
        left_fz_trajectory_end_time_ = ssp_time_start;
        left_fz_trajectory_target_ = 0.0;
      }
      else if(balancing_index_ == BalancingPhase8)
      {
        left_fz_trajectory_start_time_ = ssp_time_end;
        left_fz_trajectory_shift_ = 0.0;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_trajectory_target_ = left_dsp_fz_N_;
            left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_trajectory_target_ = 0.0;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_trajectory_target_ = left_ssp_fz_N_;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else
        {
          left_fz_trajectory_target_ = left_dsp_fz_N_;
          left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }
      }
      else
      {
        left_fz_trajectory_start_time_ = walking_time_;
        left_fz_trajectory_end_time_   = walking_time_;
      }
    }

    step_data_mutex_lock_.unlock();
    mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
        present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

    mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
        present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

    mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
        present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

    mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

    mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
    mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
    mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
    mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

    mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
    mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;


    //Stabilizer Start
    //Balancing Algorithm
    double target_fz_N  = 0;

    double right_leg_fx_N  = current_right_fx_N_;
    double right_leg_fy_N  = current_right_fy_N_;
    double right_leg_fz_N  = current_right_fz_N_;
    double right_leg_Tx_Nm = current_right_tx_Nm_;
    double right_leg_Ty_Nm = current_right_ty_Nm_;
    double right_leg_Tz_Nm = current_right_tz_Nm_;

    double left_leg_fx_N  = current_left_fx_N_;
    double left_leg_fy_N  = current_left_fy_N_;
    double left_leg_fz_N  = current_left_fz_N_;
    double left_leg_Tx_Nm = current_left_tx_Nm_;
    double left_leg_Ty_Nm = current_left_ty_Nm_;
    double left_leg_Tz_Nm = current_left_tz_Nm_;

    Eigen::MatrixXd  mat_right_force, mat_right_torque;
    mat_right_force.resize(4,1);    mat_right_force.fill(0);
    mat_right_torque.resize(4,1);   mat_right_torque.fill(0);
    mat_right_force(0,0) = right_leg_fx_N;
    mat_right_force(1,0) = right_leg_fy_N;
    mat_right_force(2,0) = right_leg_fz_N;
    mat_right_torque(0,0) = right_leg_Tx_Nm;
    mat_right_torque(1,0) = right_leg_Ty_Nm;
    mat_right_torque(2,0) = right_leg_Tz_Nm;

    Eigen::MatrixXd  mat_left_force, mat_left_torque;
    mat_left_force.resize(4,1);     mat_left_force.fill(0);
    mat_left_torque.resize(4,1);    mat_left_torque.fill(0);
    mat_left_force(0,0) = left_leg_fx_N;
    mat_left_force(1,0) = left_leg_fy_N;
    mat_left_force(2,0) = left_leg_fz_N;
    mat_left_torque(0,0) = left_leg_Tx_Nm;
    mat_left_torque(1,0) = left_leg_Ty_Nm;
    mat_left_torque(2,0) = left_leg_Tz_Nm;

    mat_right_force  = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_force;
    mat_right_torque = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_torque;

    mat_left_force  = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_force;
    mat_left_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_torque;

    imu_data_mutex_lock_.lock();
    double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec_;
    double gyro_pitch_rad_per_sec = current_gyro_pitch_rad_per_sec_;

    double iu_roll_rad  = current_imu_roll_rad_;
    double iu_pitch_rad = current_imu_pitch_rad_;
    imu_data_mutex_lock_.unlock();

    balance_ctrl_.setCurrentGyroSensorOutput(gyro_roll_rad_per_sec, gyro_pitch_rad_per_sec);
    balance_ctrl_.setCurrentOrientationSensorOutput(iu_roll_rad, iu_pitch_rad);
    balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force.coeff(0,0),  mat_right_force.coeff(1,0),  mat_right_force.coeff(2,0),
                                                        mat_right_torque.coeff(0,0), mat_right_torque.coeff(1,0), mat_right_torque.coeff(2,0),
                                                        mat_left_force.coeff(0,0),   mat_left_force.coeff(1,0),   mat_left_force.coeff(2,0),
                                                        mat_left_torque.coeff(0,0),  mat_left_torque.coeff(1,0),  mat_left_torque.coeff(2,0));


    double r_target_fx_N = 0;
    double l_target_fx_N = 0;
    double r_target_fy_N = 0;
    double l_target_fy_N = 0;
    double r_target_fz_N = right_dsp_fz_N_;
    double l_target_fz_N = left_dsp_fz_N_;

    Eigen::MatrixXd mat_g_to_acc, mat_robot_to_acc;
    mat_g_to_acc.resize(4, 1);
    mat_g_to_acc.fill(0);
    mat_g_to_acc.coeffRef(0,0) = x_lipm_.coeff(2,0);
    mat_g_to_acc.coeffRef(1,0) = y_lipm_.coeff(2,0);
    mat_robot_to_acc = mat_robot_to_g_ * mat_g_to_acc;


    switch(balancing_index_)
    {
    case BalancingPhase0:
      //fprintf(stderr, "DSP : START\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase1:
      //fprintf(stderr, "DSP : R--O->L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase2:
      //fprintf(stderr, "SSP : L_BALANCING1\n");
      r_target_fx_N = 0;
      r_target_fy_N = 0;
      r_target_fz_N = 0;

      l_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      l_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      l_target_fz_N = left_ssp_fz_N_;
      target_fz_N = left_ssp_fz_N_;
      break;
    case BalancingPhase3:
      //fprintf(stderr, "SSP : L_BALANCING2\n");
      r_target_fx_N = 0;
      r_target_fy_N = 0;
      r_target_fz_N = 0;

      l_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      l_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      l_target_fz_N = left_ssp_fz_N_;
      target_fz_N = left_ssp_fz_N_;
      break;
    case BalancingPhase4:
      //fprintf(stderr, "DSP : R--O<-L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase5:
      //fprintf(stderr, "DSP : R<-O--L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase6:
      //fprintf(stderr, "SSP : R_BALANCING1\n");
      r_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_ssp_fz_N_;

      l_target_fx_N = 0;
      l_target_fy_N = 0;
      l_target_fz_N = 0;
      target_fz_N = -right_ssp_fz_N_;
      break;
    case BalancingPhase7:
      //fprintf(stderr, "SSP : R_BALANCING2\n");
      r_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_ssp_fz_N_;

      l_target_fx_N = 0;
      l_target_fy_N = 0;
      l_target_fz_N = 0;
      target_fz_N =  -right_ssp_fz_N_;
      break;
    case BalancingPhase8:
      //fprintf(stderr, "DSP : R->O--L");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase9:
      //fprintf(stderr, "DSP : END");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    default:
      break;
    }


    bool IsDSP = false;
    if( (balancing_index_ == BalancingPhase0) ||
        (balancing_index_ == BalancingPhase1) ||
        (balancing_index_ == BalancingPhase4) ||
        (balancing_index_ == BalancingPhase5) ||
        (balancing_index_ == BalancingPhase8) ||
        (balancing_index_ == BalancingPhase9) )
    {
      IsDSP = true;
    }
    else
      IsDSP = false;

    if(IsDSP)
    {
      if( (balancing_index_ == BalancingPhase0) || (balancing_index_ == BalancingPhase9) )
      {
        r_target_fz_N = right_dsp_fz_N_;
        l_target_fz_N = left_dsp_fz_N_;
      }
      else
      {
        l_target_fz_N = wsigmoid(walking_time_ - control_time_unit_msec_*0.001, left_fz_trajectory_end_time_ -  left_fz_trajectory_start_time_, left_fz_trajectory_start_time_, left_fz_trajectory_target_ - left_fz_trajectory_shift_, left_fz_trajectory_shift_, 1.0, 1.0);
        r_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
      }
    }
    else
    {
      if( (balancing_index_ == BalancingPhase2) || (balancing_index_ == BalancingPhase3) )
      {
        r_target_fz_N = 0;
        l_target_fz_N = left_ssp_fz_N_;
      }
      else
      {
        r_target_fz_N = right_ssp_fz_N_;
        l_target_fz_N = 0;
      }
    }

    balance_ctrl_.setDesiredCOBGyro(0,0);
    balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll, present_body_pose_.pitch);
    balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
                                            l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
    balance_ctrl_.setDesiredPose(mat_robot_to_cob_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

    balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
    mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
    //Stabilizer End

    rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
    lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

    if((rhip_to_rfoot_pose_.yaw > 30.0*M_PI/180.0) || (rhip_to_rfoot_pose_.yaw < -30.0*M_PI/180.0) )
    {
      printf("yawyaw_r\n");
      return;
    }

    if((lhip_to_lfoot_pose_.yaw < -30.0*M_PI/180.0) || (lhip_to_lfoot_pose_.yaw > 30.0*M_PI/180.0) )
    {
      printf("yawyaw_l\n");
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
    {
      printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
    {
      printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
      return;
    }


    r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
    l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
    r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
    l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;


    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] + hip_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] - 0.35*hip_roll_swap;
    }

    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] + hip_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] - 0.35*hip_roll_swap;
    }

    for(int angle_idx = 0; angle_idx < 6; angle_idx++)
    {
      leg_angle_feed_back_[angle_idx+0].desired_ = r_leg_out_angle_rad_[angle_idx];
      leg_angle_feed_back_[angle_idx+6].desired_ = l_leg_out_angle_rad_[angle_idx];
      out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx] + leg_angle_feed_back_[angle_idx+0].getFeedBack(curr_angle_rad_[angle_idx]);
      out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx] + leg_angle_feed_back_[angle_idx+6].getFeedBack(curr_angle_rad_[angle_idx+6]);
    }
  }
}


double OP3OnlineWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

double OP3OnlineWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
{
  double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
  if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0))
  {
    if( time >= time_shift+period*(2-sigmoid_ratio)) {
      value = mag_shift + mag;
    }
    else
    {
      t = 2.0*M_PI*(time - time_shift)/(period*(2-sigmoid_ratio));
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if( (sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0))
  {
    if( time <= time_shift+period*(1-sigmoid_ratio))
      value = mag_shift;
    else {
      t = 2.0*M_PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if(( sigmoid_ratio >= 2.0) && ( sigmoid_ratio < 3.0))
  {
    double nsigmoid_ratio = sigmoid_ratio - 2.0;
    if(time <= time_shift + period*(1.0-nsigmoid_ratio)*0.5)
      value = mag_shift;
    else if(time >= time_shift + period*(1.0+nsigmoid_ratio)*0.5)
      value = mag + mag_shift;
    else {
      t = 2.0*M_PI*(time - (time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1.0-distortion_ratio)*(time-(time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else
    value = mag_shift;

  return value;
}

