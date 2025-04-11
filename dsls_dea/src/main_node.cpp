#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum class ControllerPhase {
    WAIT_FOR_TAKEOFF,
    DSLS_ENABLED
};

class DSLS_DEA : public rclcpp::Node
{
public:
	DSLS_DEA() : Node("dsls_dea_controller")
	{
        // ROS2 node initialization

        // initialize publishers
        // uav0
		offboard0_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory0_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle0_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        // uav1
        offboard1_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
		trajectory1_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        vehicle1_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
            switch (phase_) {
            case ControllerPhase::WAIT_FOR_TAKEOFF:
            {
                OffboardControlMode mode_msg{};
                mode_msg.position     = true;  
                mode_msg.velocity     = false;
                mode_msg.acceleration = false;
                mode_msg.attitude     = false;
                mode_msg.body_rate    = false;
                mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                offboard0_control_mode_publisher_->publish(mode_msg);
                offboard1_control_mode_publisher_->publish(mode_msg);
        
                publish_trajectory_setpoint();  
        
                if (offboard_setpoint_counter_ == 10) {
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 1); // OFFBOARD for vehicle 0
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 2); // OFFBOARD for vehicle 1
                    arm();  
                    armed_time_ = this->get_clock()->now(); 
                }
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
        
                if (is_armed_) {
                    auto now = this->get_clock()->now();
                    double elapsed = (now - armed_time_).seconds();
                    if (elapsed > 9.0) {
                        RCLCPP_INFO(this->get_logger(), "9s after arming, switching to DSLS controller");
                        //phase_ = ControllerPhase::SLS_ENABLED; // disable when building code
                    }
                }
                break;
            }
            case ControllerPhase::DSLS_ENABLED:
            {
                // // Switch to body_rate offboard mode
                // OffboardControlMode mode_msg{};
                // mode_msg.position     = !(this-> ctrl_enabled_);
                // mode_msg.velocity     = false;
                // mode_msg.acceleration = false;
                // mode_msg.attitude     = (this-> ctrl_enabled_) && (!(this-> rate_ctrl_enabled_));
                // mode_msg.body_rate    = (this-> ctrl_enabled_) && (this-> rate_ctrl_enabled_);
                // //mode_msg.thrust_and_torque = true;
                // mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                // offboard_control_mode_publisher_->publish(mode_msg);
                // this-> test = true;
                // break;
            }
            }
        };
        timer_ = this->create_wall_timer(50ms, timer_callback); // 20Hz
    }

	void arm();

private:
    // ros2 timer and time
	rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    // publishers
    // uav0
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard0_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory0_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle0_command_publisher_;
    // uav1
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard1_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory1_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle1_command_publisher_;

    // pre-takeoff
    ControllerPhase phase_ = ControllerPhase::WAIT_FOR_TAKEOFF;
    rclcpp::Time armed_time_;
    bool is_armed_ = false;
	uint64_t offboard_setpoint_counter_;   

    // methods
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_sys_id = 1);
};


void DSLS_DEA::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 1); // arm vehicle 0
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 2); // arm vehicle 1

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void DSLS_DEA::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg0{};
	msg0.position = {0.0, 0.35, -1.0}; 
	msg0.yaw = -3.14; 
	msg0.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    TrajectorySetpoint msg1{};
	msg1.position = {0.0, -0.35, -1.0};
	msg1.yaw = -3.14; 
	msg1.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	trajectory0_setpoint_publisher_->publish(msg0);
    trajectory1_setpoint_publisher_->publish(msg1);
}

void DSLS_DEA::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_sys_id)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = target_sys_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (target_sys_id == 1) {
        vehicle0_command_publisher_->publish(msg);
    } else {
        vehicle1_command_publisher_->publish(msg);
    }
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DSLS_DEA>());

	rclcpp::shutdown();
	return 0;
}
