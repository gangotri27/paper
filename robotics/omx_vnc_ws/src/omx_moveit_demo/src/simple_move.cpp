#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("omx_moveit_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("omx_moveit_demo");

  // We spin up a thread so the node can process MoveIt messages in the background
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup ARM Interface
  // "arm" is the group name we defined in the Setup Assistant
  moveit::planning_interface::MoveGroupInterface arm_group(node, "arm");
  arm_group.setMaxVelocityScalingFactor(1.0); // Full speed
  arm_group.setMaxAccelerationScalingFactor(1.0);

  // 2. Setup GRIPPER Interface
  // "gripper" is the group name we defined in the Setup Assistant
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
  gripper_group.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // --- STEP 1: Go to HOME ---
  RCLCPP_INFO(LOGGER, "Planning to HOME...");
  arm_group.setNamedTarget("home");
  
  if (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_group.execute(my_plan);
    RCLCPP_INFO(LOGGER, "Executed HOME.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to plan to HOME.");
  }

  // --- STEP 2: OPEN Gripper ---
  RCLCPP_INFO(LOGGER, "Opening Gripper...");
  // Try named target first, if you defined "open" in setup assistant
  // If "open" doesn't exist, you can set joint targets manually:
  // gripper_group.setJointValueTarget({0.019}); 
  gripper_group.setNamedTarget("open"); 
  gripper_group.move(); // move() plans and executes in one step

  // --- STEP 3: Go to PICK ---
  // Ensure you created a pose named "pick" in Setup Assistant, otherwise use "ready"
  RCLCPP_INFO(LOGGER, "Planning to READY...");
  arm_group.setNamedTarget("ready"); 
  arm_group.move();

  // --- STEP 4: CLOSE Gripper ---
  RCLCPP_INFO(LOGGER, "Closing Gripper...");
  gripper_group.setNamedTarget("close");
  gripper_group.move();

  // ... after closing gripper ...
  
  // --- STEP 5: Go to PLACE ---
  RCLCPP_INFO(LOGGER, "Planning to PLACE...");
  arm_group.setNamedTarget("place"); // Make sure "place" exists in your SRDF
  arm_group.move();

  RCLCPP_INFO(LOGGER, "Demo Complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}