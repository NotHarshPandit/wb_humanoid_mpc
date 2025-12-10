/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_wb_mpc/command/WBMpcTargetTrajectoriesCalculator.h"

#include <ocs2_core/misc/LoadData.h>

#include <boost/proto/proto_fwd.hpp>
#include <cmath>
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"
#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"

namespace ocs2::humanoid {

WBMpcTargetTrajectoriesCalculator::WBMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                     scalar_t mpcHorizon)
    : TargetTrajectoriesCalculatorBase(referenceFile, mpcRobotModel, mpcHorizon) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WBMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commadLinePoseTarget,
                                                                                            scalar_t initTime,
                                                                                            const vector_t& initState) {
  vector_t currentPose = getCurrentBasePoseTarget(initState);

  const vector_t targetPose = getDeltaBaseTarget(commadLinePoseTarget, currentPose);

  scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << currentPose, targetJointState_, vector_t::Zero(mpcRobotModelPtr_->getGenCoordinatesDim());
  stateTrajectory[1] << targetPose, targetJointState_, vector_t::Zero(mpcRobotModelPtr_->getGenCoordinatesDim());

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WBMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                                            scalar_t initTime,
                                                                                            const vector_t& initState) {
  // This function constructs a target trajectory that interpolates between the current momentum and desired momentum for the first part
  // of the horizon while applying the desired momentum fully to the latter half. All position targets are obtained integrating said
  // velocity profile.

  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);
  vector4_t commVelTargetGlobal = filterAndTransformVelCommandToLocal(commandedVelocities, currentPoseTarget(3), 0.8);

  // // Adapt desired base height from velocity command
  // currentPoseTarget[2] = commVelTargetGlobal[2];

  vector6_t targetBaseVel;
  targetBaseVel << commVelTargetGlobal(0), commVelTargetGlobal(1), 0.0, commVelTargetGlobal(3), 0.0, 0.0;

  /////////////////////////
  // Intermediate Target //
  /////////////////////////

  scalar_t intermediateTargetTime = 0.7 * mpcHorizon_;
  vector6_t baseVel = mpcRobotModelPtr_->getBaseComVelocity(initState);
  vector3_t averageVel;
  averageVel(0) = (baseVel[0] + commVelTargetGlobal[0]) / 2;
  averageVel(1) = (baseVel[1] + commVelTargetGlobal[1]) / 2;
  averageVel(2) = (baseVel[5] + commVelTargetGlobal[3]) / 2;

  currentPoseTarget[2] = commVelTargetGlobal[2];
  vector6_t intermediateTargetPose = integrateTargetBasePose(currentPoseTarget, averageVel, commVelTargetGlobal(2), intermediateTargetTime);

  //////////////////
  // Final Target //
  //////////////////

  averageVel(0) = (commVelTargetGlobal[0]);
  averageVel(1) = (commVelTargetGlobal[1]);
  averageVel(2) = (commVelTargetGlobal[3]);

  vector6_t finalTargetPose =
      integrateTargetBasePose(intermediateTargetPose, averageVel, commVelTargetGlobal(2), (mpcHorizon_ - intermediateTargetTime));

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, initTime + intermediateTargetTime, initTime + mpcHorizon_};

  // desired state trajectory
  vector_array_t stateTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << currentPoseTarget, targetJointState_, targetBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());
  stateTrajectory[1] << intermediateTargetPose, targetJointState_, targetBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());
  stateTrajectory[2] << finalTargetPose, targetJointState_, targetBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WBMpcTargetTrajectoriesCalculator::waypointToTargetTrajectories(const vector6_t& targetPose,
                                                                                   scalar_t initTime,
                                                                                   scalar_t finalTime,
                                                                                   const vector_t& initState) {
  // Get current pose from state
  vector6_t currentPose = getCurrentBasePoseTarget(initState);

  // If finalTime not specified or invalid, estimate it
  scalar_t targetReachingTime = finalTime;
  if (targetReachingTime <= initTime) {
    targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPose);
  }

  // Clamp to MPC horizon if needed
  targetReachingTime = std::min(targetReachingTime, initTime + mpcHorizon_);

  // Create intermediate waypoint for smooth transition (at 50% of trajectory)
  scalar_t intermediateTime = initTime + 0.5 * (targetReachingTime - initTime);
  vector6_t intermediatePose;
  
  // Linear interpolation for position
  intermediatePose(0) = currentPose(0) + 0.5 * (targetPose(0) - currentPose(0));  // x
  intermediatePose(1) = currentPose(1) + 0.5 * (targetPose(1) - currentPose(1));  // y
  intermediatePose(2) = currentPose(2) + 0.5 * (targetPose(2) - currentPose(2));  // z
  
  // For orientation, handle yaw separately (shortest path)
  scalar_t currentYaw = currentPose(3);
  scalar_t targetYaw = targetPose(3);
  
  // Normalize yaw difference to [-pi, pi]
  scalar_t yawDiff = targetYaw - currentYaw;
  while (yawDiff > M_PI) yawDiff -= 2.0 * M_PI;
  while (yawDiff < -M_PI) yawDiff += 2.0 * M_PI;
  
  intermediatePose(3) = currentYaw + 0.5 * yawDiff;  // yaw
  intermediatePose(4) = 0.0;  // roll (keep upright)
  intermediatePose(5) = 0.0;  // pitch (keep upright)

  // Calculate desired velocities for smooth motion
  // Average velocity over the trajectory
  vector3_t averageVel;
  averageVel(0) = (targetPose(0) - currentPose(0)) / (targetReachingTime - initTime);  // vx
  averageVel(1) = (targetPose(1) - currentPose(1)) / (targetReachingTime - initTime);  // vy
  averageVel(2) = yawDiff / (targetReachingTime - initTime);  // vyaw

  // Target base velocity (6D: vx, vy, vz, wx, wy, wz)
  vector6_t targetBaseVel;
  targetBaseVel << averageVel(0), averageVel(1), 0.0, 0.0, 0.0, averageVel(2);

  // Create time trajectory with 3 waypoints
  scalar_array_t timeTrajectory{initTime, intermediateTime, targetReachingTime};

  // Create state trajectory
  vector_array_t stateTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  
  // Initial state: current pose with target velocity
  stateTrajectory[0] << currentPose, targetJointState_, targetBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());
  
  // Intermediate state
  stateTrajectory[1] << intermediatePose, targetJointState_, targetBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());
  
  // Final state: target pose with zero velocity (arrive at rest)
  vector6_t finalBaseVel = vector6_t::Zero();
  stateTrajectory[2] << targetPose, targetJointState_, finalBaseVel, vector_t::Zero(mpcRobotModelPtr_->getJointDim());

  // Input trajectory (not used, but required)
  vector_array_t inputTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

}  // namespace ocs2::humanoid
