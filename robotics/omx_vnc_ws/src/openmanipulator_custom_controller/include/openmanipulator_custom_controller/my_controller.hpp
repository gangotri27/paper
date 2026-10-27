#ifndef OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_
#define OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openmanipulator_custom_controller
{

class MyController : public controller_interface::ControllerInterface
{
public:
  MyController() = default;

  // lifecycle hooks
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // which interfaces we need
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // main control loop
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> target_positions_;
  double step_size_ = 0.005;  // default safe step (rad) per update
};

}  // namespace openmanipulator_custom_controller

#endif  // OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_























// #ifndef OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_
// #define OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_

// #include <vector>
// #include <string>

// #include "controller_interface/controller_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// namespace openmanipulator_custom_controller
// {

// class MyController : public controller_interface::ControllerInterface
// {
// public:
//   MyController() = default;

//   // lifecycle hooks
//   controller_interface::CallbackReturn on_init() override;
//   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
//   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
//   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//   // which interfaces we need
//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;

//   // main control loop
//   controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   std::vector<std::string> joint_names_;
//   std::vector<double> target_positions_;
//   double step_size_ = 0.005;  // default safe step (rad) per update
// };

// }  // namespace openmanipulator_custom_controller

// #endif  // OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_










// #ifndef OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_
// #define OPENMANIPULATOR_CUSTOM_CONTROLLER__MY_CONTROLLER_HPP_

// #include <vector>
// #include <string>
// #include <memory>

// #include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// namespace openmanipulator_custom_controller
// {

// class MyController : public controller_interface::ControllerInterface
// {
// public:
//   MyController() = default;

//   controller_interface::CallbackReturn on_init() override;

//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;

//   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
//   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
//   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

//   controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

// private:
//   std::vector<std::string> joint_names_;
//   std::vector<double> target_positions_;
//   double step_size_ = 0.01;  // small safe step movement
// };

// }  // namespace openmanipulator_custom_controller

// #endif
