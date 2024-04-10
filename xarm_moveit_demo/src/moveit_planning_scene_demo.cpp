#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tf2/LinearMath/Quaternion.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <chrono>

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_pose_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_pose_demo");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto arm = MoveGroupInterface(node, "xarm");
  auto gripper = MoveGroupInterface(node, "gripper");

  // 创建PlanningSceneInterface对象scene用来对规划场景进行操作
  moveit::planning_interface::PlanningSceneInterface scene;
  // 设置一个比例因子以选择性地降低最大关节速度限制，可取值为(0,1]
  arm.setMaxVelocityScalingFactor(0.8);
  gripper.setMaxVelocityScalingFactor(0.8);
  // 机械臂回到初始位置
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();

  // 设置桌面对象的形状和位姿
  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = "base_link";
  table_object.id = "table";
  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[table_primitive.BOX_X] = 1.0;
  table_primitive.dimensions[table_primitive.BOX_Y] = 1.2;
  table_primitive.dimensions[table_primitive.BOX_Z] = 0.01;
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0;
  table_pose.position.y = 0;
  table_pose.position.z = -table_primitive.dimensions[2]/2.0;
  table_pose.orientation.w = 1.0;
  table_object.primitives.push_back(table_primitive);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;

  // 设置长方体对象的形状和位姿
  moveit_msgs::msg::CollisionObject box_object;
  box_object.header.frame_id = "base_link";
  box_object.id = "box";
  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = 0.25;
  box_primitive.dimensions[1] = 0.3;
  box_primitive.dimensions[2] = 0.04;
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0;
  box_pose.position.z = box_primitive.dimensions[2]/2.0 + 0.37;
  box_pose.orientation.w = 1.0;
  box_object.primitives.push_back(box_primitive);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = box_object.ADD;


  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(table_object);
  collision_objects.push_back(box_object);
  // 将桌面、长方体添加到规划场景中
  scene.addCollisionObjects(collision_objects);
  scene.applyCollisionObjects(collision_objects);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Open gripper, move the arm and close gripper...");
  // 张开手爪
  std::vector<double> gripper_joint_positions = {0.65,0.65};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();
  // 设置xarm规划组的目标位姿，让机械臂运动到该目标
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = rclcpp::Clock().now();
  target_pose.pose.position.x = 0.41;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.45;
  target_pose.pose.orientation.w = 1;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();
  // 稍微闭合手爪
  gripper_joint_positions = {0.2,0.2};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Move the arm to a position 25 cm down ...");
  // 让机械臂运动到长方体下方的位置
  target_pose.pose.position.z -= 0.25;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);

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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Remove the objects from planning scene");
  // 删除规划场景里的物体对象
  std::vector<std::string> object_ids;
  object_ids.push_back(table_object.id);
  object_ids.push_back(box_object.id);
  scene.removeCollisionObjects(object_ids);
  sleep(1);
  // 机械臂回到初始位置
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();

  rclcpp::shutdown();
  spinner.join(); 
  return 0;
}
