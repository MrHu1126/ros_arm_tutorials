#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>  
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");
namespace mtc = moveit::task_constructor;

const std::string BASE_LINK = "base_link";
const std::string TABLE_ID = "table";
const std::string TARGET_ID = "object";
const std::vector<std::string> GRIPPER_JOINT_NAMES = {"gripper_1_joint", "gripper_2_joint"};
const std::vector<double> GRIPPER_OPEN = {0.65,0.65};
const std::vector<double> GRIPPER_GRASP = {0.1,0.1};


class PickPlaceDemo
{
public:
  PickPlaceDemo(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PickPlaceDemo::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

PickPlaceDemo::PickPlaceDemo(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("moveit_pick_place_demo", options) }
{
}

void PickPlaceDemo::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


mtc::Task PickPlaceDemo::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "xarm";
  const auto& hand_group_name = "gripper";
  //const auto& hand_frame = "base_link";
  const auto& hand_frame = "gripper_centor_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "hand");
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("Open_gripper", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("Open_gripper");
  task.add(std::move(stage_open_hand));

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.2);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("Open_gripper");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 20, Eigen::Vector3d::UnitZ());
      grasp_frame_transform =  Eigen::Translation3d(0, 0, 0.03) * q;
      //grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().z() = 0.0;

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Close_gripper");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "base_link";
      target_pose_msg.pose.position.x = 0.32;
      target_pose_msg.pose.position.y = -0.32;
      target_pose_msg.pose.position.z = 0.22 / 2.0;
      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, -M_PI / 4);
      target_pose_msg.pose.orientation = tf2::toMsg(orientation);

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Open_gripper");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  return task;
}

void PickPlaceDemo::setupPlanningScene()
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = TABLE_ID;
  collision_objects[0].header.frame_id = BASE_LINK;

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.0;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.01;
  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = TARGET_ID;
  collision_objects[1].header.frame_id = BASE_LINK;

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.05;
  collision_objects[1].primitives[0].dimensions[1] = 0.05;
  collision_objects[1].primitives[0].dimensions[2] = 0.22;

  //collision_objects[1].pose.resize(1);
  collision_objects[1].pose.position.x = 0.47;
  collision_objects[1].pose.position.y = 0.0;
  collision_objects[1].pose.position.z = 0.11;
  collision_objects[1].pose.orientation.w = 1;

  collision_objects[1].operation = collision_objects[1].ADD;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_interface.applyCollisionObjects(collision_objects);
}




// void makeGripperPosture(trajectory_msgs::JointTrajectory& posture, std::vector<double> positions)
// {
//   // 设置joint_names为gripper规划组的两个关节名
//   posture.joint_names = GRIPPER_JOINT_NAMES;
//   // 设置关节的位置
//   posture.points.resize(1);
//   posture.points[0].positions = positions;
//   posture.points[0].time_from_start = ros::Duration(0.5);
// }

// void makeGrasps(std::vector<moveit_msgs::Grasp>& grasps){
//   // 设置grasps里只包含一个元素
//   grasps.resize(1);
//   // 设置抓取的位姿grasp_pose
//   grasps[0].grasp_pose.header.frame_id = BASE_LINK;
//   grasps[0].grasp_pose.pose.position.x = 0.47;
//   grasps[0].grasp_pose.pose.position.y = 0;
//   grasps[0].grasp_pose.pose.position.z = 0.14;
//   tf2::Quaternion orientation;
//   orientation.setRPY(0, 0, 0);
//   grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//   // 设置pre_grasp_approach,沿着X轴正向靠近抓取点,移动的最小距离为0.1m，期望距离为0.12米
//   grasps[0].pre_grasp_approach.direction.header.frame_id = BASE_LINK;
//   grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//   grasps[0].pre_grasp_approach.min_distance = 0.1;
//   grasps[0].pre_grasp_approach.desired_distance = 0.12;
//   // 设置post_grasp_retreat,抓取物体后，沿着Z轴正向撤离，移动的距离最小为0.08米，期望距离为0.1米
//   grasps[0].post_grasp_retreat.direction.header.frame_id = BASE_LINK;
//   grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//   grasps[0].post_grasp_retreat.min_distance = 0.08;
//   grasps[0].post_grasp_retreat.desired_distance = 0.1;
//   // 设置夹爪在抓取物品前的位姿为张开的状态
//   makeGripperPosture(grasps[0].pre_grasp_posture, GRIPPER_OPEN);
//   // 设置夹爪用于抓取对象时的位置
//   makeGripperPosture(grasps[0].grasp_posture, GRIPPER_GRASP);
// }

// void makePlaces(std::vector<moveit_msgs::PlaceLocation> &place_locations, geometry_msgs::PoseStamped init_pose){
//   // 创建moveit_msgs/PlaceLocation消息的对象place
//   moveit_msgs::PlaceLocation place;
//   // 设置放置位姿
//   place.place_pose = init_pose;
//   // 设置靠近放置点的方向、最小移动距离和期望距离，这里设置为沿着Z轴向下移动0.1米
//   place.pre_place_approach.direction.header.frame_id = BASE_LINK;
//   place.pre_place_approach.direction.vector.z = -1.0;
//   place.pre_place_approach.min_distance = 0.08;
//   place.pre_place_approach.desired_distance = 0.1;
//   // 设置放置完成后机械臂的撤离方向、移动最小距离和期望距离，这里设置为沿着Z轴向上移动0.15米
//   place.post_place_retreat.direction.header.frame_id = BASE_LINK;
//   place.post_place_retreat.direction.vector.z = 1.0;
//   place.post_place_retreat.min_distance = 0.12;
//   place.post_place_retreat.desired_distance = 0.15;
//   //  可尝试的x位置偏移量
//   std::vector<double> x_vals = {0, 0.005, 0.01, -0.005, -0.01};
//   // 在放置位置附近生成其他可放置位置，并添加到放置位姿列表place_locations
//   for (auto x:x_vals) {
//     place.place_pose.pose.position.x = init_pose.pose.position.x + x;
//     place_locations.push_back(place);
//   }
// }



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<PickPlaceDemo>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });
  RCLCPP_INFO_STREAM(LOGGER,"Pick and Place demo is ready.");

  // 添加障碍物和目标物体
  mtc_task_node->setupPlanningScene();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  mtc_task_node->doTask();


  // // 设置桌子table为抓取和放置操作的支撑面，使MoveIt!忽略物体放到桌子上时产生的碰撞警告
  // xarm_group.setSupportSurfaceName(TABLE_ID);
  // // 生成grasp抓取向量
  // std::vector<moveit_msgs::Grasp> grasps;
  // makeGrasps(grasps);
  // // 尝试进行抓取
  // ROS_INFO("Try to pick up the box. ");
  // moveit::planning_interface::MoveItErrorCode result;
  // int max_pick_attempts = 5;
  // int n_attempts = 0;
  // while(result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_pick_attempts ){
  //   n_attempts++;
  //   result = xarm_group.pick(TARGET_ID, grasps);
  //   ros::WallDuration(0.2).sleep();
  // }

  // ros::WallDuration(1.0).sleep();
  // // 设置一个放置目标位姿
  // geometry_msgs::PoseStamped place_pose;
  // place_pose.header.frame_id = BASE_LINK;
  // place_pose.pose.position.x = 0.32;
  // place_pose.pose.position.y = -0.32;
  // place_pose.pose.position.z = 0.22 / 2.0;
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, -M_PI / 4);
  // place_pose.pose.orientation = tf2::toMsg(orientation);
  // // 生成一系列放置位姿
  // std::vector<moveit_msgs::PlaceLocation> place_locations;
  // makePlaces(place_locations, place_pose);
  // // 如果抓取成功，尝试物品放置操作
  // if(result == moveit::planning_interface::MoveItErrorCode::SUCCESS){
  //   ROS_INFO("Picked up successfully. Try to place the box.");
  //   int max_place_attempts = 5;
  //   n_attempts = 0;
  //   result = 0;
  //   while(result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_place_attempts ){
  //     n_attempts++;
  //     result = xarm_group.place(TARGET_ID, place_locations);
  //     ros::WallDuration(0.2).sleep();
  //   }
  // }
  // // 闭合手爪
  // ROS_INFO("Close gripper ...");
  // gripper_group.setNamedTarget("Close_gripper");
  // gripper_group.move();
  // // 回到初始位置
  // ROS_INFO("Moving to pose: Home");
  // xarm_group.setNamedTarget("Home");
  // xarm_group.move();
  // // 删除规划场景里的桌面和目标物体
  // std::vector<std::string> object_ids = {TABLE_ID, TARGET_ID};
  // planning_scene_interface.removeCollisionObjects(object_ids);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
