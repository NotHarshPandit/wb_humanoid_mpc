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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <optional>

#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"
#include "humanoid_mpc_msgs/msg/waypoint_command.hpp"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class Ros2ProceduralMpcMotionManager : public ProceduralMpcMotionManager {
 public:
  using WaypointToTargetTrajectories = std::function<TargetTrajectories(const vector6_t& targetPose, scalar_t initTime, scalar_t finalTime, const vector_t& initState)>;

  Ros2ProceduralMpcMotionManager(const std::string& gaitFile,
                                 const std::string& referenceFile,
                                 std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories,
                                 WaypointToTargetTrajectories waypointToTargetTrajectories = nullptr);

  ~Ros2ProceduralMpcMotionManager() override = default;

  /** Disable copy / move */
  Ros2ProceduralMpcMotionManager& operator=(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager& operator=(Ros2ProceduralMpcMotionManager&&) = delete;
  Ros2ProceduralMpcMotionManager(Ros2ProceduralMpcMotionManager&&) = delete;

  void setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) override;

  void setWaypointCommand(const humanoid_mpc_msgs::msg::WaypointCommand& waypointCommand);

  void subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos);

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

 private:
  WalkingVelocityCommand getScaledWalkingVelocityCommand() override;

  bool hasWaypointCommand() const;
  void clearWaypointCommand();

  rclcpp::Subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr velCommandSubscriber_;
  rclcpp::Subscription<humanoid_mpc_msgs::msg::WaypointCommand>::SharedPtr waypointCommandSubscriber_;
  
  std::mutex walkingVelCommandMutex_;
  mutable std::mutex waypointCommandMutex_;  // mutable to allow locking in const member functions
  
  std::optional<humanoid_mpc_msgs::msg::WaypointCommand> waypointCommand_;
  vector6_t absoluteTargetPose_;  // Absolute target pose (computed once when waypoint is first received)
  bool absoluteTargetPoseSet_ = false;  // Flag to indicate if absolute target has been computed
  scalar_t waypointStartTime_ = -1.0;  // Time when waypoint was first set (-1 means not set)
  scalar_t previousDistance_ = -1.0;  // Previous distance to target (-1 means not set)
  scalar_t lastProgressCheckTime_ = -1.0;  // Time of last progress check
  scalar_t waypointStopTime_ = -1.0;  // Time when waypoint was reached and robot should stop (-1 means not stopping)
  static constexpr scalar_t MAX_WAYPOINT_TIME = 30.0;  // Maximum time to pursue waypoint (30 seconds)
  static constexpr scalar_t PROGRESS_CHECK_INTERVAL = 2.0;  // Check progress every 2 seconds
  static constexpr scalar_t MIN_PROGRESS_RATE = 0.05;  // Minimum progress rate (m/s) to continue
  static constexpr scalar_t STOP_TRAJECTORY_DURATION = 3.0;  // Duration to maintain stop trajectory after waypoint reached
  WaypointToTargetTrajectories waypointToTargetTrajectoriesFun_;
};

}  // namespace ocs2::humanoid
