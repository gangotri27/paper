#include "openmanipulator_custom_controller/my_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"  // HW_IF_POSITION
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

using hardware_interface::HW_IF_POSITION;

namespace openmanipulator_custom_controller
{

controller_interface::CallbackReturn MyController::on_init()
{
  // declare parameter using the node API so it exists for later calls
  // note: get_node() returns a raw rclcpp node pointer in controller base
  get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  // request individual command interfaces for the joints we control
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // If joint_names_ is empty try to read from parameter (const method -> use const_cast)
  if (joint_names_.empty()) {
    std::vector<std::string> tmp;
    const_cast<MyController*>(this)->get_node()->get_parameter("joints", tmp);
    for (const auto & j : tmp) {
      cfg.names.push_back(j + "/" + std::string(HW_IF_POSITION));
    }
  } else {
    for (const auto & j : joint_names_) {
      cfg.names.push_back(j + "/" + std::string(HW_IF_POSITION));
    }
  }

  return cfg;
}

controller_interface::InterfaceConfiguration MyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (joint_names_.empty()) {
    std::vector<std::string> tmp;
    const_cast<MyController*>(this)->get_node()->get_parameter("joints", tmp);
    for (const auto & j : tmp) {
      cfg.names.push_back(j + "/" + std::string(HW_IF_POSITION));
    }
  } else {
    for (const auto & j : joint_names_) {
      cfg.names.push_back(j + "/" + std::string(HW_IF_POSITION));
    }
  }

  return cfg;
}

controller_interface::CallbackReturn MyController::on_configure(const rclcpp_lifecycle::State &)
{
  // Read joints parameter from node
  get_node()->get_parameter("joints", joint_names_);

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "MyController: 'joints' parameter is empty. Set joints in YAML.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  target_positions_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "MyController configured for %zu joints", joint_names_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_activate(const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() < joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "MyController: not enough state interfaces available");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // populate initial target_positions_ from state interfaces (they are ordered as requested)
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double cur = 0.0;
    // state_interfaces_ elements expose get_optional()
    auto opt = state_interfaces_[i].get_optional();
    if (opt.has_value()) {
      cur = opt.value();
    }
    target_positions_[i] = cur;
  }

  RCLCPP_INFO(get_node()->get_logger(), "MyController activated; initial targets set.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "MyController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MyController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n = joint_names_.size();
  if (command_interfaces_.size() < n || state_interfaces_.size() < n) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "MyController: interface size mismatch (cmd:%zu state:%zu expected:%zu)",
      command_interfaces_.size(), state_interfaces_.size(), n);
    return controller_interface::return_type::OK;
  }

  for (size_t i = 0; i < n; ++i) {
    double current = 0.0;
    auto opt = state_interfaces_[i].get_optional();
    if (opt.has_value()) {
      current = opt.value();
    }

    double error = target_positions_[i] - current;
    double cmd = current;

    if (std::fabs(error) > step_size_) {
      double step = (error > 0.0) ? step_size_ : -step_size_;
      cmd = current + step;
    } else {
      cmd = target_positions_[i];
    }

    // set_value exists on command interface wrapper
    bool ok = command_interfaces_[i].set_value(cmd);
    (void)ok;
  }

  return controller_interface::return_type::OK;
}

}  // namespace openmanipulator_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openmanipulator_custom_controller::MyController, controller_interface::ControllerInterface)
