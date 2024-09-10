#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/roi.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control_two")
	{
		node_namespace = this->get_namespace();


		size_t underscore_pos = node_namespace.find('_');
		
		if (underscore_pos != std::string::npos) {
			// Extract the substring starting after the underscore
			std::string number_str = node_namespace.substr(underscore_pos + 1);
			
			// Convert the extracted substring to an integer
			int extracted_number = std::stoi(number_str);
			
			// When ROS 2 nodes want to send VehicleCommand to PX4, 
			// they must ensure that the messages are filled with the appropriate target_system value.
			// For example, if you want to send a command to your third vehicle, 
			// which has px4_instance=2, you need to set target_system=3 in all your VehicleCommand messages.
			target_system = extracted_number + 1;
		} else {
			std::cerr << "No underscore found in the node_namespace string." << std::endl;
		}

        // Node publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(node_namespace+"/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(node_namespace+"/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(node_namespace+"/fmu/in/vehicle_command", 10);

		// Node subscribers
		// Configure QoS profile for publishing and subscribing
		rclcpp::QoS qos_profile(1);  // depth = 1
		qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
		qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

		vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(node_namespace+"/fmu/out/vehicle_odometry", qos_profile,
		                                                                                   std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1));
		
		roi_subscriber_ = this->create_subscription<px4_msgs::msg::Roi>("/roi", qos_profile, 
		                                                               std::bind(&OffboardControl::roi_callback, this, std::placeholders::_1));

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			if (feedback_position[2] > -5.0) {
				publish_trajectory_setpoint({0.0, 0.0, -5.0}, -3.14); // [-PI, PI]
			}
			else {
				if (feedback_position[0] <= 5.0) {
					if (target_system == 2) {
						publish_trajectory_setpoint({5.0, 0.0, -5.0}, -3.14); // [-PI, PI]
					}
					else {
						publish_trajectory_setpoint({0.0, -5.0, -5.0}, -3.14); // [-PI, PI]
					}
				}
				else {
					RCLCPP_INFO(this->get_logger(), "Land");
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
					exit(0);
				}
			}

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_local_position_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::Roi>::SharedPtr roi_subscriber_;


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(const std::array<float, 3UL>& position, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void odometry_callback(const px4_msgs::msg::VehicleOdometry& vlp);
	void roi_callback(const px4_msgs::msg::Roi& roi);

	std::array<float, 3UL> feedback_position;
	std::array<std::array<float, 3UL>, 4UL> roi_;
	std::string node_namespace;
	uint8_t target_system;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(const std::array<float, 3UL>& position, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = position;
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.target_system = target_system;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry& vlp)
{
	feedback_position[0] = vlp.position[0];
	feedback_position[1] = vlp.position[1];
	feedback_position[2] = vlp.position[2];
	//RCLCPP_INFO(this->get_logger(), "OffboardControl::odometry_callback %f", vlp.position[2]);
}

void OffboardControl::roi_callback(const px4_msgs::msg::Roi& roi)
{
	// vertex-1
	roi_[0][0] = roi.vertex_1[0];
	roi_[0][1] = roi.vertex_1[1];
	roi_[0][2] = roi.vertex_1[2];
	// vertex-2
	roi_[1][0] = roi.vertex_2[0];
	roi_[1][1] = roi.vertex_2[1];
	roi_[1][2] = roi.vertex_2[2];
	// vertex-3
	roi_[2][0] = roi.vertex_3[0];
	roi_[2][1] = roi.vertex_3[1];
	roi_[2][2] = roi.vertex_3[2];
	// vertex-4
	roi_[3][0] = roi.vertex_4[0];
	roi_[3][1] = roi.vertex_4[1];
	roi_[3][2] = roi.vertex_4[2];

	RCLCPP_INFO(this->get_logger(), "OffboardControl::roi_callback");
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
