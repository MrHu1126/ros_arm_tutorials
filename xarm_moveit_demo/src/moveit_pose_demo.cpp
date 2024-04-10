#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tf2/LinearMath/Quaternion.h"
#include <thread>        
#include <chrono> 
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
  // 获取xarm规划组的规划参考坐标系
  std::string planning_frame = arm.getPlanningFrame();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Planning frame : "<< planning_frame);
  // 获取末端执行器的link
  std::string eef_link = arm.getEndEffectorLink();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "End effector link : "<< eef_link);
  // 若allow_replanning()参数为True，则MoveIt!在一次规划失败后进行重新规划
  arm.allowReplanning(true);
  // 设置运动到目标时的位置(单位：米)和姿态的容忍误差(单位：弧度)
  arm.setGoalPositionTolerance(0.02);
  arm.setGoalOrientationTolerance(0.03);
  // 设置一个比例因子以选择性地降低最大关节速度限制，可取值为(0,1]
  arm.setMaxVelocityScalingFactor(0.8);
  // 使用geometry_msgs/PoseStamped消息类型设置机械臂的目标位姿
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = planning_frame;
  target_pose.header.stamp = rclcpp::Clock().now();
  target_pose.pose.position.x = 0.3;
  target_pose.pose.position.y = 0.1;
  target_pose.pose.position.z = 0.25;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 3.1415926/2.0,0);
  target_pose.pose.orientation.x = quaternion.x();
  target_pose.pose.orientation.y = quaternion.y();
  target_pose.pose.orientation.z = quaternion.z();
  target_pose.pose.orientation.w = quaternion.w();
  // 显示地把开始状态设置为机械臂的当前状态
  arm.setStartStateToCurrentState();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Moving to target_pose ...");
  // 设置目标位姿
  arm.setPoseTarget(target_pose);
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

  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // 获取末端执行器当前的位姿
  geometry_msgs::msg::PoseStamped current_pose = arm.getCurrentPose();
  // 获取机械臂六个关节当前的位置
  std::vector<double> current_joint_positions =  arm.getCurrentJointValues();
  // 设定新的目标为当前位置的正前方（X轴正向）5厘米处的位置。
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Move forward 5 cm  ...");
  target_pose = current_pose;
  target_pose.pose.position.x += 0.05;
  arm.setPoseTarget(target_pose);
  arm.move();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}

