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

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <cmath>
#include <iomanip>

#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc/command/WalkingVelocityCommand.h"
#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

Ros2ProceduralMpcMotionManager::Ros2ProceduralMpcMotionManager(
    const std::string& gaitFile,
    const std::string& referenceFile,
    std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
    const MpcRobotModelBase<scalar_t>& mpcRobotModel,
    VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories,
    WaypointToTargetTrajectories waypointToTargetTrajectories)
    : ProceduralMpcMotionManager(
          gaitFile, referenceFile, switchedModelReferenceManagerPtr, mpcRobotModel, velocityTargetToTargetTrajectories),
      waypointToTargetTrajectoriesFun_(std::move(waypointToTargetTrajectories)) {}

void Ros2ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
}

void Ros2ProceduralMpcMotionManager::setWaypointCommand(const humanoid_mpc_msgs::msg::WaypointCommand& waypointCommand) {
  std::lock_guard<std::mutex> lock(waypointCommandMutex_);
  waypointCommand_ = waypointCommand;
  waypointStartTime_ = -1.0;  // Reset start time, will be set in preSolverRun
  absoluteTargetPoseSet_ = false;  // Reset absolute target flag, will be computed in preSolverRun
  waypointStopTime_ = -1.0;  // Clear stop time when new waypoint is set
}

void Ros2ProceduralMpcMotionManager::subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos) {
  // ModeSchedule

  // TargetTrajectories - Velocity command
  auto walkingVelocityCallback = [this](const humanoid_mpc_msgs::msg::WalkingVelocityCommand::SharedPtr msg) {
    this->setAndScaleVelocityCommand(getWalkingVelocityCommandFromMsg(*msg));
    // Clear waypoint command when velocity command is received
    this->clearWaypointCommand();
  };
  velCommandSubscriber_ = nodeHandle->create_subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>(
      "humanoid/walking_velocity_command", qos, walkingVelocityCallback);

  // Waypoint command
  auto waypointCallback = [this](const humanoid_mpc_msgs::msg::WaypointCommand::SharedPtr msg) {
    // Ignore new waypoint commands if one is already active
    if (this->hasWaypointCommand()) {
      std::cerr << "\n[Ros2ProceduralMpcMotionManager] WARNING: Ignoring new waypoint command - "
                << "another waypoint is already active!" << std::endl;
      return;
    }
    
    std::cerr << "\n[Ros2ProceduralMpcMotionManager] Received waypoint command:" << std::endl;
    std::cerr << "  Target (relative): x=" << msg->target_x << ", y=" << msg->target_y 
              << ", z=" << msg->target_z << ", yaw=" << msg->target_yaw << std::endl;
    std::cerr << "  Motion type: " << msg->motion_type << std::endl;
    std::cerr << "  Time to reach: " << msg->time_to_reach << "s" << std::endl;
    this->setWaypointCommand(*msg);
    std::cerr << "  Waypoint command stored successfully" << std::endl;
  };
  waypointCommandSubscriber_ = nodeHandle->create_subscription<humanoid_mpc_msgs::msg::WaypointCommand>(
      "humanoid/waypoint_command", qos, waypointCallback);
}

void Ros2ProceduralMpcMotionManager::preSolverRun(scalar_t initTime,
                                                   scalar_t finalTime,
                                                   const vector_t& initState,
                                                   const ReferenceManagerInterface& referenceManager) {
  // Print current position at EVERY iteration (before any waypoint processing)
  vector6_t currentPose = mpcRobotModelPtr_->getBasePose(initState);
  std::cerr << "[POS] t=" << std::fixed << std::setprecision(2) << initTime << "s | "
            << "x=" << std::setprecision(3) << currentPose(0) 
            << ", y=" << currentPose(1) << ", z=" << currentPose(2) 
            << ", yaw=" << currentPose(5) << std::endl;
  std::cerr.flush();
  
  // Check if waypoint command is available and waypoint function is provided
  if (hasWaypointCommand() && waypointToTargetTrajectoriesFun_) {
    std::lock_guard<std::mutex> lock(waypointCommandMutex_);
    const auto& waypointCmd = waypointCommand_.value();
    
    // Track when waypoint was first set and compute absolute target ONCE
    if (waypointStartTime_ < 0.0 || !absoluteTargetPoseSet_) {
      waypointStartTime_ = initTime;
      previousDistance_ = -1.0;
      lastProgressCheckTime_ = initTime;
      
      // Compute absolute target pose ONCE when waypoint is first received
      // Interpret waypoint as RELATIVE to the position when command was received
      absoluteTargetPose_(0) = currentPose(0) + waypointCmd.target_x;  // relative x
      absoluteTargetPose_(1) = currentPose(1) + waypointCmd.target_y;  // relative y
      absoluteTargetPose_(2) = waypointCmd.target_z;  // absolute height
      absoluteTargetPose_(3) = 0.0;  // roll=0 (upright)
      absoluteTargetPose_(4) = 0.0;  // pitch=0 (upright)
      absoluteTargetPose_(5) = currentPose(5) + waypointCmd.target_yaw;  // relative yaw
      absoluteTargetPoseSet_ = true;
      
      std::cerr << "\n[Ros2ProceduralMpcMotionManager] Waypoint command started at t=" << initTime << std::endl;
      std::cerr << "  Relative waypoint: x=" << waypointCmd.target_x << ", y=" << waypointCmd.target_y 
                << ", z=" << waypointCmd.target_z << ", yaw=" << waypointCmd.target_yaw << std::endl;
      std::cerr << "  Initial pose: x=" << currentPose(0) << ", y=" << currentPose(1) 
                << ", z=" << currentPose(2) << ", yaw=" << currentPose(5) << std::endl;
      std::cerr << "  Absolute target: x=" << absoluteTargetPose_(0) << ", y=" << absoluteTargetPose_(1) 
                << ", z=" << absoluteTargetPose_(2) << ", yaw=" << absoluteTargetPose_(5) << std::endl;
    }
    
    // Log current position IMMEDIATELY at the start of processing (before any calculations)
    std::cerr << "\n[Ros2ProceduralMpcMotionManager] Processing waypoint command (t=" << initTime << "s):" << std::endl;
    std::cerr << "  >>> Current position: x=" << currentPose(0) << ", y=" << currentPose(1) 
              << ", z=" << currentPose(2) << ", yaw=" << currentPose(5) << std::endl;
    std::cerr << "  Absolute target: x=" << absoluteTargetPose_(0) << ", y=" << absoluteTargetPose_(1) 
              << ", z=" << absoluteTargetPose_(2) << ", yaw=" << absoluteTargetPose_(5) << std::endl;
    std::cerr << "  Time to reach: " << waypointCmd.time_to_reach << "s" << std::endl;
    
    // Use the fixed absolute target pose (computed once)
    vector6_t targetPose = absoluteTargetPose_;
    
    // Check if robot is close to target FIRST, before generating trajectory
    vector6_t poseError = targetPose - currentPose;
    scalar_t positionError = poseError.head<3>().norm();
    
    // Normalize yaw error to [-pi, pi] range
    scalar_t yawError = poseError(5);  // yaw is at index 5
    while (yawError > M_PI) yawError -= 2.0 * M_PI;
    while (yawError < -M_PI) yawError += 2.0 * M_PI;
    yawError = std::abs(yawError);
    
    // Get current base velocity for speed check
    vector6_t baseVelocity = mpcRobotModelPtr_->getBaseComVelocity(initState);
    scalar_t linearSpeed = baseVelocity.head<3>().norm();
    scalar_t angularSpeed = std::abs(baseVelocity(5));  // yaw velocity
    
    // Check if we should stop BEFORE generating trajectory
    // Use balanced thresholds: tighter than original (0.2-0.4m) but achievable
    bool shouldStop = false;
    if (positionError < 0.10 && yawError < 0.15) {
      shouldStop = true;
      std::cerr << "\n*** TARGET REACHED! (distance=" << positionError << "m < 0.10m) ***" << std::endl;
    } else if (positionError < 0.15 && linearSpeed < 0.08 && angularSpeed < 0.08) {
      shouldStop = true;
      std::cerr << "\n*** TARGET REACHED (slow motion)! (distance=" << positionError 
                << "m, speed=" << linearSpeed << "m/s) ***" << std::endl;
    } else if (positionError < 0.25 && linearSpeed < 0.03 && angularSpeed < 0.03) {
      shouldStop = true;
      std::cerr << "\n*** TARGET REACHED (very slow)! (distance=" << positionError 
                << "m, speed=" << linearSpeed << "m/s) ***" << std::endl;
    }
    
    if (shouldStop) {
      // Clear waypoint command immediately
      std::cerr << "  Clearing waypoint command and STOPPING NOW." << std::endl;
      std::cerr << "  Current: x=" << currentPose(0) << ", y=" << currentPose(1) << std::endl;
      std::cerr << "  Target: x=" << targetPose(0) << ", y=" << targetPose(1) << std::endl;
      waypointCommand_.reset();
      waypointStartTime_ = -1.0;
      absoluteTargetPoseSet_ = false;
      previousDistance_ = -1.0;
      waypointStopTime_ = initTime;
      
      // Transition to stance gait IMMEDIATELY
      currentGaitMode_ = 0;  // stance
      GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
      GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
      lastGaitCommand_ = currentGaitCommand_;
      std::cerr << "  Forced stance gait." << std::endl;
      
      // Create AGGRESSIVE stop trajectory at CURRENT position with multiple waypoints
      // to ensure the MPC sees the stop command over its entire horizon
      vector6_t stopPose = currentPose;  // Stop where we are NOW, not at target
      vector6_t zeroVel = vector6_t::Zero();
      vector_t currentJointAngles = mpcRobotModelPtr_->getJointAngles(initState);
      vector_t zeroJointVel = vector_t::Zero(mpcRobotModelPtr_->getJointDim());
      
      // Create trajectory with multiple points to fill the MPC horizon
      scalar_t stopDuration = finalTime - initTime;  // Use full MPC horizon
      size_t numPoints = 5;  // Multiple waypoints to emphasize stopping
      scalar_array_t stopTimeTrajectory;
      vector_array_t stopStateTrajectory;
      vector_array_t stopInputTrajectory;
      
      for (size_t i = 0; i < numPoints; ++i) {
        scalar_t t = initTime + (stopDuration * i / (numPoints - 1));
        stopTimeTrajectory.push_back(t);
        
        vector_t state = vector_t::Zero(mpcRobotModelPtr_->getStateDim());
        state << currentPose, currentJointAngles, zeroVel, zeroJointVel;
        stopStateTrajectory.push_back(state);
        
        stopInputTrajectory.push_back(vector_t::Zero(mpcRobotModelPtr_->getInputDim()));
      }
      
      TargetTrajectories stopTrajectory{stopTimeTrajectory, stopStateTrajectory, stopInputTrajectory};
      switchedModelReferenceManagerPtr_->setTargetTrajectories(stopTrajectory);
      
      std::cerr << "  Set AGGRESSIVE stop trajectory with " << numPoints << " waypoints at current position." << std::endl;
      std::cerr << "  Stop position: x=" << stopPose(0) << ", y=" << stopPose(1) 
                << ", z=" << stopPose(2) << std::endl;
      return;
    }
    
    // Not at target yet, generate trajectory to target
    // Calculate final time
    scalar_t finalTimeForWaypoint = waypointCmd.time_to_reach > 0.0 
        ? initTime + waypointCmd.time_to_reach 
        : finalTime;
    
    std::cerr << "  Generating trajectory from current state to target..." << std::endl;
    
    // Generate target trajectory from waypoint
    TargetTrajectories targetTrajectories = waypointToTargetTrajectoriesFun_(
        targetPose, initTime, finalTimeForWaypoint, initState);
    
    std::cerr << "  Trajectory generated with " << targetTrajectories.timeTrajectory.size() 
              << " waypoints" << std::endl;
    std::cerr << "  Setting target trajectories in reference manager..." << std::endl;
    
    switchedModelReferenceManagerPtr_->setTargetTrajectories(targetTrajectories);
    
    // IMMEDIATELY force a walking gait if robot is in stance and needs to move
    // This ensures the robot starts moving right away
    if (positionError > 0.1 && currentGaitCommand_ == "stance") {
      std::cerr << "  [IMMEDIATE] Robot in stance but needs to move. Forcing walk gait NOW." << std::endl;
      currentGaitMode_ = 2;  // walk
      GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
      GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
      lastGaitCommand_ = currentGaitCommand_;
      lastGaitChangeTime_ = initTime;
      std::cerr << "  [IMMEDIATE] Gait schedule updated to: " << currentGaitCommand_ << std::endl;
    }
    
    // Log position details with error calculation
    std::cerr << "  >>> Current position: x=" << currentPose(0) << ", y=" << currentPose(1) 
              << ", z=" << currentPose(2) << ", yaw=" << currentPose(5) << std::endl;
    std::cerr << "  Target position: x=" << targetPose(0) << ", y=" << targetPose(1) 
              << ", z=" << targetPose(2) << ", yaw=" << targetPose(5) << std::endl;
    std::cerr << "  Position error: dx=" << poseError(0) << ", dy=" << poseError(1) 
              << ", dz=" << poseError(2) << ", distance=" << positionError << "m" << std::endl;
    std::cerr << "  Yaw error: " << yawError << "rad" << std::endl;
    std::cerr.flush();  // Force flush to ensure output appears immediately
    
    // Update gait schedule to enable walking for waypoint following
    // Calculate desired velocity from waypoint to determine appropriate gait
    scalar_t timeToReach = std::max(finalTimeForWaypoint - initTime, 0.1);
    vector3_t desiredVel;
    desiredVel(0) = (targetPose(0) - currentPose(0)) / timeToReach;
    desiredVel(1) = (targetPose(1) - currentPose(1)) / timeToReach;
    // Calculate yaw velocity (normalize yaw difference first)
    scalar_t yawDiff = targetPose(5) - currentPose(5);
    while (yawDiff > M_PI) yawDiff -= 2.0 * M_PI;
    while (yawDiff < -M_PI) yawDiff += 2.0 * M_PI;
    desiredVel(2) = yawDiff / timeToReach;
    
    // Normalize velocities to [-1, 1] range for gait selection (similar to velocity command system)
    // Use max velocities from config
    vector4_t velCommandVec;
    velCommandVec(0) = std::clamp(desiredVel(0) / maxDisplacementVelocityX_, -1.0, 1.0);
    velCommandVec(1) = std::clamp(desiredVel(1) / maxDisplacementVelocityY_, -1.0, 1.0);
    velCommandVec(2) = targetPose(2);  // height
    velCommandVec(3) = std::clamp(desiredVel(2) / maxRotationVelocity_, -1.0, 1.0);
    
    std::cerr << "  Desired velocity (normalized): vx=" << velCommandVec(0) 
              << ", vy=" << velCommandVec(1) << ", vyaw=" << velCommandVec(3) << std::endl;
    
    // Use baseVelocity already computed above (line 159)
    // Update gait schedule based on desired velocity (similar to velocity-based system)
    GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
    
    // If there's significant movement required, ensure we're not in stance
    scalar_t linearVelMag = std::sqrt(velCommandVec(0) * velCommandVec(0) + velCommandVec(1) * velCommandVec(1));
    if (linearVelMag > 0.1) {
      // For waypoint following, use faster gait by default
      // Calculate which gait is appropriate based on velocity magnitude
      size_t targetGaitMode = 1;  // Start with slow_walk
      
      if (linearVelMag > 0.8) {
        targetGaitMode = 5;  // trot
      } else if (linearVelMag > 0.65) {
        targetGaitMode = 4;  // slow_trot
      } else if (linearVelMag > 0.45) {
        targetGaitMode = 3;  // slower_trot
      } else if (linearVelMag > 0.25) {
        targetGaitMode = 2;  // walk
      } else {
        targetGaitMode = 1;  // slow_walk
      }
      
      // Only change if we're in stance or need to go faster
      if (currentGaitCommand_ == "stance" || currentGaitMode_ < targetGaitMode) {
        std::cerr << "  Movement required (vel_mag=" << linearVelMag 
                  << "), transitioning to gait mode " << targetGaitMode << std::endl;
        currentGaitMode_ = targetGaitMode;
        currentCfg = gaitModeStates_[currentGaitMode_];
        currentGaitCommand_ = currentCfg.gaitCommand;
        lastGaitChangeTime_ = initTime;
      }
    }
    
    // Do not change the gait pattern too frequently
    if (initTime > lastGaitChangeTime_ + 0.2) {
      if (transitionToFasterGait(velCommandVec, baseVelocity, currentCfg)) {
        std::cerr << "  Increasing to faster gait: " << currentCfg.gaitCommand << std::endl;
        currentGaitMode_++;
        currentCfg = gaitModeStates_[currentGaitMode_];
        currentGaitCommand_ = currentCfg.gaitCommand;
        lastGaitChangeTime_ = initTime;
      } else if (transitionToSlowerGait(velCommandVec, baseVelocity, currentCfg)) {
        std::cerr << "  Decreasing to slower gait: " << currentCfg.gaitCommand << std::endl;
        currentGaitMode_--;
        currentCfg = gaitModeStates_[currentGaitMode_];
        currentGaitCommand_ = currentCfg.gaitCommand;
        lastGaitChangeTime_ = initTime;
      }
    }
    
    // Update gait schedule if needed
    if (currentGaitCommand_ != lastGaitCommand_) {
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
      GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
      lastGaitCommand_ = currentGaitCommand_;
      std::cerr << "  Updated gait schedule to: " << currentGaitCommand_ << std::endl;
    } else if (positionError > 0.1) {
      // If we're not at target and gait hasn't changed, make sure we have a walking gait
      if (currentGaitCommand_ == "stance") {
        std::cerr << "  Warning: Still in stance but need to move. Forcing walk gait." << std::endl;
        currentGaitMode_ = 2;  // walk
        currentCfg = gaitModeStates_[currentGaitMode_];
        currentGaitCommand_ = currentCfg.gaitCommand;
        ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
        GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
        lastGaitCommand_ = currentGaitCommand_;
        std::cerr << "  Forced gait schedule to: " << currentGaitCommand_ << std::endl;
      }
    }
    
    // Check for timeout: if waypoint has been active for too long, clear it
    scalar_t waypointElapsedTime = initTime - waypointStartTime_;
    bool waypointTimeout = (waypointElapsedTime > MAX_WAYPOINT_TIME);
    
    // Check if robot is making progress towards target
    bool noProgress = false;
    if (previousDistance_ > 0.0 && (initTime - lastProgressCheckTime_) >= PROGRESS_CHECK_INTERVAL) {
      scalar_t distanceChange = previousDistance_ - positionError;
      scalar_t timeElapsed = initTime - lastProgressCheckTime_;
      scalar_t progressRate = distanceChange / timeElapsed;
      
      std::cerr << "  Progress check: distance change=" << distanceChange << "m over " 
                << timeElapsed << "s (rate=" << progressRate << "m/s)" << std::endl;
      
      // If not making sufficient progress and still far from target, consider stopping
      if (progressRate < MIN_PROGRESS_RATE && positionError > 0.3) {
        std::cerr << "  Warning: Insufficient progress (rate=" << progressRate 
                  << "m/s < " << MIN_PROGRESS_RATE << "m/s)" << std::endl;
        // Don't stop yet, but log the warning
      }
      
      lastProgressCheckTime_ = initTime;
    }
    previousDistance_ = positionError;
    
    // Use existing baseVelocity to check if robot is moving slowly (already computed above)
    // Check for timeout only (stopping is handled earlier)
    bool waypointReached = false;
    if (waypointTimeout) {
      std::cerr << "\n*** WAYPOINT TIMEOUT! ***" << std::endl;
      std::cerr << "  Time elapsed: " << waypointElapsedTime << "s > " << MAX_WAYPOINT_TIME << "s" << std::endl;
      std::cerr << "  Clearing waypoint command and stopping." << std::endl;
      waypointCommand_.reset();
      waypointStartTime_ = -1.0;
      absoluteTargetPoseSet_ = false;
      previousDistance_ = -1.0;
      waypointStopTime_ = initTime;
      waypointReached = true;
      
      // Stop at current position
      currentGaitMode_ = 0;  // stance
      GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
      GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
      lastGaitCommand_ = currentGaitCommand_;
      
      vector6_t stopPose = currentPose;
      vector6_t zeroVel = vector6_t::Zero();
      vector_t currentJointAngles = mpcRobotModelPtr_->getJointAngles(initState);
      vector_t zeroJointVel = vector_t::Zero(mpcRobotModelPtr_->getJointDim());
      scalar_t stopDuration = std::min(finalTime - initTime, 3.0);
      scalar_array_t stopTimeTrajectory{initTime, initTime + stopDuration};
      vector_array_t stopStateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
      stopStateTrajectory[0] << currentPose, currentJointAngles, zeroVel, zeroJointVel;
      stopStateTrajectory[1] << stopPose, currentJointAngles, zeroVel, zeroJointVel;
      vector_array_t stopInputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));
      TargetTrajectories stopTrajectory{stopTimeTrajectory, stopStateTrajectory, stopInputTrajectory};
      switchedModelReferenceManagerPtr_->setTargetTrajectories(stopTrajectory);
      return;
    } else {
      std::cerr << "  Robot not at target yet:" << std::endl;
      std::cerr << "    - Distance: " << positionError << "m (threshold: 0.10-0.25m)" << std::endl;
      std::cerr << "    - Yaw error: " << yawError << "rad (threshold: 0.15rad)" << std::endl;
      std::cerr << "    - Linear speed: " << linearSpeed << "m/s" << std::endl;
      std::cerr << "    - Keeping waypoint command active." << std::endl;
    }
    
    // Waypoint still active, continue processing it
    return;
  }
  
  // Check if we recently stopped at a waypoint and need to maintain stop trajectory
  if (waypointStopTime_ >= 0.0) {
    scalar_t timeSinceStop = initTime - waypointStopTime_;
    if (timeSinceStop < STOP_TRAJECTORY_DURATION) {
      // Continue maintaining stop trajectory
      vector6_t currentPose = mpcRobotModelPtr_->getBasePose(initState);
      vector6_t zeroVel = vector6_t::Zero();
      vector_t currentJointAngles = mpcRobotModelPtr_->getJointAngles(initState);
      vector_t zeroJointVel = vector_t::Zero(mpcRobotModelPtr_->getJointDim());
      
      scalar_t remainingStopTime = STOP_TRAJECTORY_DURATION - timeSinceStop;
      scalar_t stopEndTime = initTime + remainingStopTime;
      scalar_array_t stopTimeTrajectory{initTime, stopEndTime};
      vector_array_t stopStateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
      stopStateTrajectory[0] << currentPose, currentJointAngles, zeroVel, zeroJointVel;
      stopStateTrajectory[1] << currentPose, currentJointAngles, zeroVel, zeroJointVel;
      vector_array_t stopInputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));
      
      TargetTrajectories stopTrajectory{stopTimeTrajectory, stopStateTrajectory, stopInputTrajectory};
      switchedModelReferenceManagerPtr_->setTargetTrajectories(stopTrajectory);
      
      // Log current position while stopped
      std::cerr << "  [STOPPED] Current position: x=" << currentPose(0) << ", y=" << currentPose(1) 
                << ", z=" << currentPose(2) << ", yaw=" << currentPose(5) << std::endl;
      std::cerr << "  Maintaining stop trajectory (remaining: " << remainingStopTime << "s)" << std::endl;
      
      // Ensure stance gait
      if (currentGaitCommand_ != "stance") {
        currentGaitMode_ = 0;
        GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
        currentGaitCommand_ = currentCfg.gaitCommand;
        ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);
        GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
        lastGaitCommand_ = currentGaitCommand_;
      }
      
      return;
    } else {
      // Stop period expired, clear the flag
      std::cerr << "  Stop period expired, resuming normal operation." << std::endl;
      waypointStopTime_ = -1.0;
    }
  }
  
  // Fall back to default velocity-based trajectory generation
  ProceduralMpcMotionManager::preSolverRun(initTime, finalTime, initState, referenceManager);
}

WalkingVelocityCommand Ros2ProceduralMpcMotionManager::getScaledWalkingVelocityCommand() {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  return velocityCommand_;
}

bool Ros2ProceduralMpcMotionManager::hasWaypointCommand() const {
  std::lock_guard<std::mutex> lock(waypointCommandMutex_);
  return waypointCommand_.has_value();
}

void Ros2ProceduralMpcMotionManager::clearWaypointCommand() {
  std::lock_guard<std::mutex> lock(waypointCommandMutex_);
  waypointCommand_.reset();
  waypointStartTime_ = -1.0;
  absoluteTargetPoseSet_ = false;
  previousDistance_ = -1.0;
  lastProgressCheckTime_ = -1.0;
  waypointStopTime_ = -1.0;
}

}  // namespace ocs2::humanoid
