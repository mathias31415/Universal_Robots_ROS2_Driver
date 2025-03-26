#pragma once

#include "ur_robot_driver/hardware_interface.hpp"
#include <bitset>
#include <vector>
#include <array>

namespace ur_robot_driver
{

class URStateHelper
{
public:
    URStateHelper(
        urcl::vector6d_t& urcl_joint_positions,
        urcl::vector6d_t& urcl_joint_velocities,
        urcl::vector6d_t& urcl_joint_efforts,
        urcl::vector6d_t& urcl_ft_sensor_measurements,
        std::bitset<18>& actual_dig_out_bits,
        std::bitset<18>& actual_dig_in_bits,
        std::bitset<11>& safety_status_bits,
        std::bitset<4>& analog_io_types,
        std::bitset<4>& robot_status_bits,
        std::bitset<2>& tool_analog_input_types,
        std::array<double, 2>& tool_analog_input,
        std::array<double, 2>& standard_analog_input,
        std::array<double, 2>& standard_analog_output,
        int32_t& tool_output_voltage,
        int32_t& robot_mode,
        int32_t& safety_mode,
        uint32_t& tool_mode,
        double& tool_output_current,
        double& tool_temperature,
        double& speed_scaling_combined,
        bool& initialized,
        bool& robot_program_running,
        urcl::vector6d_t& urcl_tcp_pose,
        double& tcp_rotation_buffer_x,
        double& tcp_rotation_buffer_y,
        double& tcp_rotation_buffer_z,
        double& tcp_rotation_buffer_w,
        double& get_robot_software_version_major,
        double& get_robot_software_version_minor,
        double& get_robot_software_version_bugfix,
        double& get_robot_software_version_build,
        std::array<double, 18>& actual_dig_out_bits_copy,
        std::array<double, 18>& actual_dig_in_bits_copy,
        std::array<double, 11>& safety_status_bits_copy,
        std::array<double, 4>& analog_io_types_copy,
        std::array<double, 4>& robot_status_bits_copy,
        std::array<double, 2>& tool_analog_input_types_copy,
        double& tool_output_voltage_copy,
        double& robot_mode_copy,
        double& safety_mode_copy,
        double& tool_mode_copy,
        double& system_interface_initialized,
        double& robot_program_running_copy
    ) :
        urcl_joint_positions_(urcl_joint_positions),
        urcl_joint_velocities_(urcl_joint_velocities),
        urcl_joint_efforts_(urcl_joint_efforts),
        urcl_ft_sensor_measurements_(urcl_ft_sensor_measurements),
        actual_dig_out_bits_(actual_dig_out_bits),
        actual_dig_in_bits_(actual_dig_in_bits),
        safety_status_bits_(safety_status_bits),
        analog_io_types_(analog_io_types),
        robot_status_bits_(robot_status_bits),
        tool_analog_input_types_(tool_analog_input_types),
        tool_analog_input_(tool_analog_input),
        standard_analog_input_(standard_analog_input),
        standard_analog_output_(standard_analog_output),
        tool_output_voltage_(tool_output_voltage),
        robot_mode_(robot_mode),
        safety_mode_(safety_mode),
        tool_mode_(tool_mode),
        tool_output_current_(tool_output_current),
        tool_temperature_(tool_temperature),
        speed_scaling_combined_(speed_scaling_combined),
        initialized_(initialized),
        robot_program_running_(robot_program_running),
        urcl_tcp_pose_(urcl_tcp_pose),
        tcp_rotation_buffer_x_(tcp_rotation_buffer_x),
        tcp_rotation_buffer_y_(tcp_rotation_buffer_y),
        tcp_rotation_buffer_z_(tcp_rotation_buffer_z),
        tcp_rotation_buffer_w_(tcp_rotation_buffer_w),
        get_robot_software_version_major_(get_robot_software_version_major),
        get_robot_software_version_minor_(get_robot_software_version_minor),
        get_robot_software_version_bugfix_(get_robot_software_version_bugfix),
        get_robot_software_version_build_(get_robot_software_version_build),
        actual_dig_out_bits_copy_(actual_dig_out_bits_copy),
        actual_dig_in_bits_copy_(actual_dig_in_bits_copy),
        safety_status_bits_copy_(safety_status_bits_copy),
        analog_io_types_copy_(analog_io_types_copy),
        robot_status_bits_copy_(robot_status_bits_copy),
        tool_analog_input_types_copy_(tool_analog_input_types_copy),
        tool_output_voltage_copy_(tool_output_voltage_copy),
        robot_mode_copy_(robot_mode_copy),
        safety_mode_copy_(safety_mode_copy),
        tool_mode_copy_(tool_mode_copy),
        system_interface_initialized_(system_interface_initialized),
        robot_program_running_copy_(robot_program_running_copy)
    {
    }

    std::vector<hardware_interface::StateInterface> generate_state_interfaces(
        std::vector<std::string>& joint_names,
        std::string& tf_prefix,
        std::vector<std::string>& sensor_names)
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Joints
        for (size_t i = 0; i < joint_names.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
        }

        // Speed scaling
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

        // Force-torque sensor
        for (const auto& sensor : sensor_names) {
            if (sensor == tf_prefix + "tcp_fts_sensor") {
                const std::vector<std::string> fts_names = {
                    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
                };
                for (uint j = 0; j < 6; ++j) {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor, fts_names[j], &urcl_ft_sensor_measurements_[j]));
                }
            }
        }

        // GPIO
        for (size_t i = 0; i < 18; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits_copy_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
        }

        for (size_t i = 0; i < 11; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits_copy_[i]));
        }

        for (size_t i = 0; i < 4; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_copy_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits_copy_[i]));
        }
       
        for (size_t i = 0; i < 2; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_copy_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
        }

        // Other states
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_copy_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_copy_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_copy_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_copy_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized", &system_interface_initialized_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running_copy_));

        // TCP pose
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.x", &urcl_tcp_pose_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.y", &urcl_tcp_pose_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.z", &urcl_tcp_pose_[2]));

        // TCP rotation
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.x", &tcp_rotation_buffer_x_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.y", &tcp_rotation_buffer_y_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.z", &tcp_rotation_buffer_z_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.w", &tcp_rotation_buffer_w_));

        // Software version
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_major", &get_robot_software_version_major_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_minor", &get_robot_software_version_minor_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_bugfix", &get_robot_software_version_bugfix_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_build", &get_robot_software_version_build_));

        return state_interfaces;
    }

    void update_non_double_values()
    {
        for (size_t i = 0; i < 18; ++i) {
            actual_dig_out_bits_copy_[i] = static_cast<double>(actual_dig_out_bits_[i]);
            actual_dig_in_bits_copy_[i] = static_cast<double>(actual_dig_in_bits_[i]);
        }

        for (size_t i = 0; i < 11; ++i) {
            safety_status_bits_copy_[i] = static_cast<double>(safety_status_bits_[i]);
        }

        for (size_t i = 0; i < 4; ++i) {
            analog_io_types_copy_[i] = static_cast<double>(analog_io_types_[i]);
            robot_status_bits_copy_[i] = static_cast<double>(robot_status_bits_[i]);
        }

        for (size_t i = 0; i < 2; ++i) {
            tool_analog_input_types_copy_[i] = static_cast<double>(tool_analog_input_types_[i]);
        }

        tool_output_voltage_copy_ = static_cast<double>(tool_output_voltage_);
        robot_mode_copy_ = static_cast<double>(robot_mode_);
        safety_mode_copy_ = static_cast<double>(safety_mode_);
        tool_mode_copy_ = static_cast<double>(tool_mode_);
        system_interface_initialized_ = initialized_ ? 1.0 : 0.0;
        robot_program_running_copy_ = robot_program_running_ ? 1.0 : 0.0;
    }

private:
    urcl::vector6d_t& urcl_joint_positions_;
    urcl::vector6d_t& urcl_joint_velocities_;
    urcl::vector6d_t& urcl_joint_efforts_;
    urcl::vector6d_t& urcl_ft_sensor_measurements_;
    std::bitset<18>& actual_dig_out_bits_;
    std::bitset<18>& actual_dig_in_bits_;
    std::bitset<11>& safety_status_bits_;
    std::bitset<4>& analog_io_types_;
    std::bitset<4>& robot_status_bits_;
    std::bitset<2>& tool_analog_input_types_;
    std::array<double, 2>& tool_analog_input_;
    std::array<double, 2>& standard_analog_input_;
    std::array<double, 2>& standard_analog_output_;
    int32_t& tool_output_voltage_;
    int32_t& robot_mode_;
    int32_t& safety_mode_;
    uint32_t& tool_mode_;
    double& tool_output_current_;
    double& tool_temperature_;
    double& speed_scaling_combined_;
    bool& initialized_;
    bool& robot_program_running_;
    urcl::vector6d_t& urcl_tcp_pose_;
    double& tcp_rotation_buffer_x_;
    double& tcp_rotation_buffer_y_;
    double& tcp_rotation_buffer_z_;
    double& tcp_rotation_buffer_w_;
    double& get_robot_software_version_major_;
    double& get_robot_software_version_minor_;
    double& get_robot_software_version_bugfix_;
    double& get_robot_software_version_build_;

    std::array<double, 18>& actual_dig_out_bits_copy_;
    std::array<double, 18>& actual_dig_in_bits_copy_;
    std::array<double, 11>& safety_status_bits_copy_;
    std::array<double, 4>& analog_io_types_copy_;
    std::array<double, 4>& robot_status_bits_copy_;
    std::array<double, 2>& tool_analog_input_types_copy_;
    double& tool_output_voltage_copy_;
    double& robot_mode_copy_;
    double& safety_mode_copy_;
    double& tool_mode_copy_;
    double& system_interface_initialized_;
    double& robot_program_running_copy_;
};

}  // namespace ur_robot_driver