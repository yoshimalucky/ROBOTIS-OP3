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


/* Author: Jay Song */

#ifndef OP3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_
#define OP3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "op3_online_walking_module_msgs/AddStepDataArray.h"
#include "op3_foot_step_generator/Step2DArray.h"

#define STOP_WALKING           (0)
#define FORWARD_WALKING        (1)
#define BACKWARD_WALKING       (2)
#define RIGHTWARD_WALKING      (3)
#define LEFTWARD_WALKING       (4)
#define LEFT_ROTATING_WALKING  (5)
#define RIGHT_ROTATING_WALKING (6)

#define MINIMUM_STEP_TIME_SEC  (0.3)

namespace robotis_op2
{

class FootStepGenerator
{
public:
  FootStepGenerator();
  ~FootStepGenerator();

  void initialize();

  void calcRightKickStep(op3_online_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const op3_online_walking_module_msgs::StepData& ref_step_data);
  void calcLeftKickStep(op3_online_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const op3_online_walking_module_msgs::StepData& ref_step_data);

  void getStepData(op3_online_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const op3_online_walking_module_msgs::StepData& ref_step_data,
      int desired_step_type);

  void getStepDataFromStepData2DArray(op3_online_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const op3_online_walking_module_msgs::StepData& ref_step_data,
      const op3_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);

  int    num_of_step_;
  double fb_step_length_m_;
  double rl_step_length_m_;
  double rotate_step_angle_rad_;

  double step_time_sec_;
  double start_end_time_sec_;
  double dsp_ratio_;

  double foot_z_swap_m_;
  double body_z_swap_m_;

  double default_y_feet_offset_m_;

private:
  bool calcStep(const op3_online_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);

  void calcFBStep(const op3_online_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRLStep(const op3_online_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRoStep(const op3_online_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcStopStep(const op3_online_walking_module_msgs::StepData& ref_step_data, int direction);

  Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
  void getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw);
  op3_online_walking_module_msgs::PoseXYZRPY getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform);
  Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);

  op3_online_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

  int previous_step_type_;
};

}

#endif /* OP3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_ */
