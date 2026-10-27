#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ex11_planning_scene");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ex11_planning_scene_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup
  moveit::planning_interface::MoveGroupInterface arm_group(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  arm_group.setMaxVelocityScalingFactor(0.5);
  arm_group.setMaxAccelerationScalingFactor(0.5);
  arm_group.setPlanningTime(10.0);

  // ---------------------------------------------------------
  // 1. CLEANUP & SPAWN TABLE
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Cleaning up old objects...");
  planning_scene_interface.removeCollisionObjects({"table", "cube"});
  std::this_thread::sleep_for(std::chrono::seconds(1));

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = arm_group.getPlanningFrame();
  table.id = "table";
  shape_msgs::msg::SolidPrimitive table_prim;
  table_prim.type = table_prim.BOX;
  table_prim.dimensions = {0.5, 0.5, 0.05}; 
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.6; // Keep safe distance
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.20; 
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_prim);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  RCLCPP_INFO(LOGGER, "Adding Table...");
  planning_scene_interface.applyCollisionObjects({table});
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ---------------------------------------------------------
  // 2. MOVE TO PICK (Empty Handed)
  // ---------------------------------------------------------

  RCLCPP_INFO(LOGGER, "Moving to HOME");
  arm_group.setNamedTarget("home");
  arm_group.move();

  RCLCPP_INFO(LOGGER, "Opening Gripper");
  std::vector<double> gripper_vals;
  gripper_vals.push_back(0.019); 
  gripper_group.setJointValueTarget(gripper_vals);
  gripper_group.move();

  RCLCPP_INFO(LOGGER, "Moving to PICK pose (Empty)...");
  arm_group.setNamedTarget("pick");
  
  if (arm_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Failed to reach Pick pose.");
      return 1;
  }

  // ---------------------------------------------------------
  // 3. SPAWN CUBE (VISUAL FIX HERE)
  // ---------------------------------------------------------
  
  RCLCPP_INFO(LOGGER, "Reached Pick. Spawning Cube at FINGERTIPS...");
  
  geometry_msgs::msg::Pose current_pose = arm_group.getCurrentPose().pose;
  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = arm_group.getPlanningFrame();
  cube.id = "cube";
  shape_msgs::msg::SolidPrimitive cube_prim;
  cube_prim.type = cube_prim.BOX;
  cube_prim.dimensions = {0.02, 0.02, 0.02}; 
  
  geometry_msgs::msg::Pose cube_pose = current_pose;
  
  // --- THE FIX ---
  // Previous value (0.02) put it inside the wrist motor (link4/link5 junction).
  // We increase this to 0.07 (7cm) to push it out to where the fingers are.
  // Note: On OpenManipulator, X points "forward" out of the gripper.
  cube_pose.position.x += 0.07; 

  cube.primitives.push_back(cube_prim);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;

  planning_scene_interface.applyCollisionObjects({cube});
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ---------------------------------------------------------
  // 4. ATTACH
  // ---------------------------------------------------------

  RCLCPP_INFO(LOGGER, "Attaching Cube...");
  // Whitelist link4/link5 so MoveIt doesn't complain about the "Wrist" being close
  arm_group.attachObject(cube.id, "link5", {
      "link5", 
      "link4", 
      "end_effector_link",
      "gripper_left_link", 
      "gripper_right_link"
  });

  RCLCPP_INFO(LOGGER, "Closing Gripper");
  gripper_vals[0] = -0.005; 
  gripper_group.setJointValueTarget(gripper_vals);
  gripper_group.move();

  // ---------------------------------------------------------
  // 5. PLACE (Split Moves)
  // ---------------------------------------------------------

  RCLCPP_INFO(LOGGER, "Lifting (Shoulder)...");
  std::vector<double> joints = arm_group.getCurrentJointValues();
  joints[1] -= 0.5; // Lift
  arm_group.setJointValueTarget(joints);
  if (arm_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(LOGGER, "Lift failed. Checking collisions...");
  }

  RCLCPP_INFO(LOGGER, "Turning (Base)...");
  joints = arm_group.getCurrentJointValues(); 
  joints[0] += 1.0; // Turn
  arm_group.setJointValueTarget(joints);
  if (arm_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(LOGGER, "Turn failed.");
  }

  // ---------------------------------------------------------
  // 6. DETACH & HOME
  // ---------------------------------------------------------

  RCLCPP_INFO(LOGGER, "Detaching Cube...");
  arm_group.detachObject(cube.id);

  RCLCPP_INFO(LOGGER, "Returning HOME");
  arm_group.setNamedTarget("home");
  arm_group.move();

  RCLCPP_INFO(LOGGER, "Exercise 11 COMPLETE.");
  rclcpp::shutdown();
  return 0;
}