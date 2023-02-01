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

/* Author: Sachin Chitta, Michael Lautman */

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
      rclcpp::Node::make_shared("motion_planning_api_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_api_tutorial_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string PLANNING_GROUP = "panda_arm";
  robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!motion_planning_api_tutorial_node->get_parameter("planning_plugin", planner_plugin_name))
    RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, motion_planning_api_tutorial_node,
                                      motion_planning_api_tutorial_node->get_namespace()))
      RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                 ex.what(), ss.str().c_str());
  }

  // moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  
  // Create timer
  std::chrono::steady_clock::time_point const time_start = std::chrono::steady_clock::now();
  size_t const NUM_ATTEMPTS = 100;
  for (size_t counter = 0; counter < NUM_ATTEMPTS; counter++)
  {
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      return 0;
    }
  }
  size_t elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
  RCLCPP_WARN_STREAM(LOGGER, " Elapsed Time: " <<  elapsed_time_ms / NUM_ATTEMPTS << " ms");


  // std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
  //     motion_planning_api_tutorial_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
  //                                                                                              1);
  // moveit_msgs::msg::DisplayTrajectory display_trajectory;

  // /* Visualize the trajectory */
  // moveit_msgs::msg::MotionPlanResponse response;
  // res.getMessage(response);

  // for (auto const& pt: response.trajectory.joint_trajectory.points)
  // {
  //   RCLCPP_WARN_STREAM(LOGGER, pt.positions[0] << ", " << pt.positions[1] << ", " << pt.positions[2] << ", " << pt.positions[3] << ", " << pt.positions[4] << ", " << pt.positions[5]);
  // }

  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  // display_publisher->publish(display_trajectory);

  // /* Set the state in the planning scene to the final state of the last plan */
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());

  planner_instance.reset();

  rclcpp::shutdown();
  return 0;
}
