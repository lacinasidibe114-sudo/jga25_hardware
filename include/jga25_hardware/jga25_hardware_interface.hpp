#ifndef JGA25_HARDWARE__JGA25_HARDWARE_INTERFACE_HPP_
#define JGA25_HARDWARE__JGA25_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <libserial/SerialPort.h> // Bibliothèque incluse

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jga25_hardware
{

class JGA25HardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort serial_port_; 
  
  std::string device_;
  int baud_rate_ = 115200;
  double counts_per_rev_ = 1320.0;
  double rad_per_tick_ = 0.0;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
};

}  // namespace jga25_hardware

#endif  // JGA25_HARDWARE__JGA25_HARDWARE_INTERFACE_HPP_