#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <cmath>

namespace arms_hardware_interface {
class ArmsHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Add private members for state and command interfaces
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;
    

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> actuator_positions_;
    std::vector<double> actuator_velocities_;
    std::vector<double> joint_position_commands_;
    std::vector<double> joint_velocity_commands_;
    std::vector<double> actuator_pos_commands_;
    std::vector<double> actuator_vel_commands_;
    
    // HardwareInfo and other members
    hardware_interface::HardwareInfo info_;
    
    // Add any additional private methods or members needed for the implementation
    void init_variables();
    
    // Helper functions for angle conversion
    double degToRad(double degrees) const {
      return degrees * M_PI / 180.0;
    }
    
    double radToDeg(double radians) const {
      return radians * 180.0 / M_PI;
    }

}; 

} // namespace arms_hardware_interface
