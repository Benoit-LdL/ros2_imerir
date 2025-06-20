#include "imerir_manip/imerir_manip.h"

#include <memory>
#include <chrono>
#include <thread>
#include <array>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // Create a shared pointer for the node and enable automatic parameter declaration
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a logger for logging messages
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface for the "arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Create the MoveIt MoveGroup Interface for the "gripper" planning group
  auto gripper_interface = MoveGroupInterface(node, "gripper");

  // Define the target pose for the robot arm
  auto target_pose = [](const auto& pos) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = pos[3];  // Orientation (quaternion x)
    msg.orientation.y = pos[4];  // Orientation (quaternion y)
    msg.orientation.z = pos[5];  // Orientation (quaternion z)
    msg.orientation.w = pos[6];  // Orientation (quaternion w)
    msg.position.x = pos[0];   // Position in x
    msg.position.y = pos[1];   // Position in y
    msg.position.z = pos[2];   // Position in z
    return msg;
  };

  std::vector<std::array<float, 7>> mov_pos = {
    {0.162, 0.000, 0.185, -0.000, 0.000, 0.000, 1.000},     // origin
    {0.244, -0.000, 0.031, 0.000, 0.356, -0.000, 0.935},    // first pos
    {0.162, 0.000, 0.185, -0.000, 0.000, 0.000, 1.000},     // origin
    {-0.082, -0.155, 0.056, -0.319, -0.180, 0.810, -0.458}, // second pos
    {0.162, 0.000, 0.185, -0.000, 0.000, 0.000, 1.000}      // origin
  };

  
  // sleep for a few seconds
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // Open the gripper
  gripper_interface.setNamedTarget("open");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
  } else {
    RCLCPP_ERROR(logger, "Failed to open the gripper");
  }
  
  for(int i = 0; i < static_cast<int>(mov_pos.size()); ++i) {
    // Set the target pose for the arm
    move_group_interface.setPoseTarget(target_pose(mov_pos[i]));

    // Set tolerances for goal position and orientation
    move_group_interface.setGoalPositionTolerance(0.02);
    move_group_interface.setGoalOrientationTolerance(0.02);

    // Plan the motion for the arm to reach the target pose
    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg) == moveit::core::MoveItErrorCode::SUCCESS);
      return std::make_pair(ok, msg);
    }();

    // If planning succeeds, execute the planned motion
    if (success) {
      move_group_interface.execute(plan);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(logger, "Planning failed for the arm!");  // Log an error if planning fails
    }

    if(i == 1) {
      // Close the gripper
      gripper_interface.setNamedTarget("close");
      if (gripper_interface.move()) {
        RCLCPP_INFO(logger, "Gripper closed successfully");  // Log success
      } else {
        RCLCPP_ERROR(logger, "Failed to close the gripper");
      }
    }


    if(i == static_cast<int>(mov_pos.size())-2) {
        // Open the gripper
      gripper_interface.setNamedTarget("open");
      if (gripper_interface.move()) {
        RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
      } else {
        RCLCPP_ERROR(logger, "Failed to open the gripper");
      }
    }

  }

  // Shut down the ROS2 node
  rclcpp::shutdown();
  return 0;
}