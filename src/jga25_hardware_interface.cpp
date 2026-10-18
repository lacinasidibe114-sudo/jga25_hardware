#include "jga25_hardware/jga25_hardware_interface.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace jga25_hardware
{

hardware_interface::CallbackReturn JGA25HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Lecture des paramètres URDF
  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  counts_per_rev_ = std::stod(info_.hardware_parameters["counts_per_rev"]);
  rad_per_tick_ = (2.0 * M_PI) / counts_per_rev_;

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JGA25HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    // OUVERTURE DU PORT SÉRIE
    serial_port_.Open(device_);
    
    // Configuration du baudrate
    if (baud_rate_ == 115200) {
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } else if (baud_rate_ == 9600) {
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("JGA25Hardware"), "Baudrate non supporté: %d", baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Configuration de la trame (8N1)
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    RCLCPP_INFO(rclcpp::get_logger("JGA25Hardware"), "Port %s ouvert avec libserial.", device_.c_str());
    
  } catch (const LibSerial::OpenFailed&) {
    RCLCPP_FATAL(rclcpp::get_logger("JGA25Hardware"), "Impossible d'ouvrir le port %s", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JGA25HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_port_.IsOpen()) {
    try {
      serial_port_.Write("V0\n");
    } catch (...) {}
    
    serial_port_.Close();
    RCLCPP_INFO(rclcpp::get_logger("JGA25Hardware"), "Port fermé.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JGA25HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JGA25HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ==========================================
// FONCTION READ
// ==========================================
hardware_interface::return_type JGA25HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

  try {
    std::string data;
    // Lecture non-bloquante avec timeout de 5ms 
    // (si l'Arduino n'a rien envoyé, on passe au cycle suivant)
    serial_port_.ReadLine(data, '\n', 5); 
    
    // Parsing : on s'attend à "P1234 V567\n"
    long ticks_pos = 0;
    long ticks_vel = 0;
    if (std::sscanf(data.c_str(), "P%ld V%ld", &ticks_pos, &ticks_vel) == 2) {
      hw_positions_[0]  = static_cast<double>(ticks_pos) * rad_per_tick_;
      hw_velocities_[0] = static_cast<double>(ticks_vel) * rad_per_tick_;
    }
  } catch (const LibSerial::ReadTimeout&) {
    // Timeout normal, on ignore
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("JGA25Hardware"), "Erreur lecture série: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ==========================================
// FONCTION WRITE
// ==========================================
hardware_interface::return_type JGA25HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

  try {
    // Conversion Rad/s -> Ticks/s
    long target_ticks_vel = static_cast<long>(hw_commands_[0] / rad_per_tick_);
    
    // Formatage de la commande : "V123\n"
    std::string cmd = "V" + std::to_string(target_ticks_vel) + "\n";
    
    serial_port_.Write(cmd);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("JGA25Hardware"), "Erreur écriture série: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace jga25_hardware

PLUGINLIB_EXPORT_CLASS(
  jga25_hardware::JGA25HardwareInterface,
  hardware_interface::SystemInterface)