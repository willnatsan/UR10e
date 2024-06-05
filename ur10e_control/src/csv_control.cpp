#include <fstream>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <utility> // std::pair
#include <vector>

int x, y;

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "csv_control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("csv_control");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur10e_arm");

  // Create an input filestream
  std::ifstream myFile("9x9_sample.csv");

  // Make sure the file is open
  if (!myFile.is_open())
    throw std::runtime_error("Could not open file");

  // Helper vars
  std::string line, colname;
  int val;

  if (myFile.good()) {
    std::getline(myFile, line); // Extract the first line in the file
    std::stringstream ss(line); // Create a stringstream from line
  }

  // Read data, line by line
  while (std::getline(myFile, line)) {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Keep track of the current column index
    int colIdx = 0;

    // Extract just the first two columns (x and y coordinates)
    while (colIdx < 2) {

      if (colIdx == 0) {
        ss >> x;
        RCLCPP_INFO(logger, "x: %d", x);
      } else {
        ss >> y;
        RCLCPP_INFO(logger, "y: %d", y);
      }

      // If the next token is a comma, ignore it and move on
      if (ss.peek() == ',')
        ss.ignore();

      // Set a target Pose
      auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = 0.5;
        return msg;
      }();
      move_group_interface.setPoseTarget(target_pose);

      // Create a plan to that target pose
      auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      // Execute the plan
      if (success) {
        move_group_interface.execute(plan);
      } else {
        RCLCPP_ERROR(logger, "Planning failed!");
      }

      // Increment the column index
      colIdx++;
    }
  }

  // Shutdown
  myFile.close();
  rclcpp::shutdown();
  return 0;
}