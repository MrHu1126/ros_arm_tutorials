#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_joint_pose_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_joint_pose_demo");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto arm = MoveGroupInterface(node, "xarm");
  auto gripper = MoveGroupInterface(node, "gripper");

  arm.setGoalJointTolerance(0.01);
  gripper.setGoalJointTolerance(0.01);
  arm.setMaxVelocityScalingFactor(0.8);
  gripper.setMaxVelocityScalingFactor(0.8);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Moveing to joint-space goal: joint_positions");

  std::vector<double> arm_joint_positions = {-0.664, -0.775, 0.675, -1.241, -0.473, -1.281};
  arm.setJointValueTarget(arm_joint_positions);
    // Create a plan to that target pose
  auto const [success, plan] = [&arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Open gripper ...");
  std::vector<double> gripper_joint_positions = {0.65,0.65};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Close gripper ...");
  gripper.setNamedTarget("Close_gripper");
  gripper.move();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
