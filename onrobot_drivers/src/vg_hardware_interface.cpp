#include "onrobot_driver/vg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vg_hardware_interface
{

    VGHardwareInterface::VGHardwareInterface()
        : vacuum_state_(0.0),
          vacuum_command_(0.0)
    {
    }

    VGHardwareInterface::~VGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Retrieve the gripper type from the hardware parameters.
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for valid gripper type.
        if (onrobot_type_ != "vgc10" && onrobot_type_ != "vg10")
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Invalid onrobot_type: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the connection type from the hardware parameters.
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for TCP connection parameters.
        if (connection_type_ == "tcp")
        {
            if (info.hardware_parameters.find("ip_address") != info.hardware_parameters.end() &&
                info.hardware_parameters.find("port") != info.hardware_parameters.end())
            {
                ip_address_ = info.hardware_parameters.at("ip_address");
                port_ = std::stoi(info.hardware_parameters.at("port"));
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // Check for Serial connection parameters.
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") != info.hardware_parameters.end())
            {
                device_ = info.hardware_parameters.at("device");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the prefix from the hardware parameters.
        if (info.hardware_parameters.find("prefix") != info.hardware_parameters.end())
        {
            prefix_ = info.hardware_parameters.at("prefix");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialise joint variables
        vacuum_state_ = 0.0;
        vacuum_command_ = 0.0;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        // Create the RG instance.
        try
        {
            if (connection_type_ == "tcp")
            {
                gripper_ = std::make_unique<VG>(onrobot_type_, ip_address_, port_);
            }
            else if (connection_type_ == "serial")
            {
                gripper_ = std::make_unique<VG>(onrobot_type_, device_);
            }
            // Get the starting vacuum level of the gripper.
            vacuum_state_ = gripper_->getVacuumLevel(VG::STATUS_ADDR_A_VACUUM);
            vacuum_command_ = vacuum_state_;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Failed to create VG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
            gripper_.release();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> VGHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "_suction_regulator_joint", "position", &vacuum_state_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> VGHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "_suction_regulator_joint", "position", &vacuum_command_));
        return command_interfaces;
    }

    hardware_interface::return_type VGHardwareInterface::read(const rclcpp::Time &,
                                                              const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            vacuum_state_ = gripper_->getVacuumLevel(VG::STATUS_ADDR_A_VACUUM);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Failed to read gripper state: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VGHardwareInterface::write(const rclcpp::Time &,
                                                               const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            // TODO: MODIFY FOR VG
            gripper_->setVacuumLevel(vacuum_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

} // namespace vg_hardware_interface

// Export the hardware interface as a plugin for ros2_control.
PLUGINLIB_EXPORT_CLASS(vg_hardware_interface::VGHardwareInterface, hardware_interface::ActuatorInterface)
