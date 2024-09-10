#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_with_obstacle_avoidance")
    {
        // Declare parameters for obstacle position
        this->declare_parameter<float>("obstacle_x", 0.0);
        this->declare_parameter<float>("obstacle_y", 0.0);
        this->declare_parameter<float>("obstacle_z", 3.0);

        // Node publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Configure QoS profile for publishing and subscribing
		rclcpp::QoS qos_profile(1);  // depth = 1
		qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
		qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        // Subscriber to get the vehicle's current position
        vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile, std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1));

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void
        {
            if (offboard_setpoint_counter_ == 10)
            {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // Update obstacle position from parameters
            update_obstacle_position();

            // Plan trajectory to avoid obstacle and reach goal
            auto [x, y, z] = plan_trajectory();
            publish_offboard_control_mode();
            if (feedback_position_[2] > -3.0) {
				publish_trajectory_setpoint({0.0, 0.0, -3.0}, -3.14); // [-PI, PI]
			}
            else {
                publish_trajectory_setpoint({x, y, z}, -3.14);
            }

            // Stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11)
            {
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

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    std::array<float, 3UL> feedback_position_;
    std::array<float, 3UL> obstacle_position_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const std::array<float, 3UL> &position, float yaw);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void odometry_callback(const px4_msgs::msg::VehicleOdometry &vlp);

    void update_obstacle_position();
    std::tuple<float, float, float> plan_trajectory();

    std::array<float, 3UL> goal_position_{0.0, 0.0, -5.0}; // Example goal position
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
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
 */
void OffboardControl::publish_trajectory_setpoint(const std::array<float, 3UL> &position, float yaw)
{
    TrajectorySetpoint msg{};
    msg.position = position;
    msg.yaw = yaw; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
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
    vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Callback to update the drone's current position
 */
void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry &vlp)
{
    feedback_position_[0] = vlp.position[0];
    feedback_position_[1] = vlp.position[1];
    feedback_position_[2] = vlp.position[2];
}

/**
 * @brief Update obstacle position from ROS2 parameters
 */
void OffboardControl::update_obstacle_position()
{
    obstacle_position_[0] = this->get_parameter("obstacle_x").as_double();
    obstacle_position_[1] = this->get_parameter("obstacle_y").as_double();
    obstacle_position_[2] = this->get_parameter("obstacle_z").as_double();
}

/**
 * @brief Simple trajectory planning to avoid obstacles
 */
std::tuple<float, float, float> OffboardControl::plan_trajectory()
{
    // Current position
    float x = feedback_position_[0];
    float y = feedback_position_[1];
    float z = feedback_position_[2];

    //std::cout << "position: " << x << " " << y << " " << z << std::endl;

    // Obstacle avoidance parameters
    float obstacle_avoidance_gain = 3.0; // Gain to avoid the obstacle

    // Calculate vector towards goal
    float goal_x = goal_position_[0] - x;
    float goal_y = goal_position_[1] - y;
    float goal_z = goal_position_[2] - z;

    // Calculate repulsive force from obstacle
    float dist_to_obstacle = sqrt(pow(obstacle_position_[0] - x, 2) +
                                  pow(obstacle_position_[1] - y, 2) +
                                  pow(obstacle_position_[2] - z, 2));

    //std::cout << "dist_to_obstacle: " << dist_to_obstacle << std::endl;

    if (dist_to_obstacle < 2.0) // Threshold distance for avoidance
    {
        float repel_x = (x - obstacle_position_[0]) / pow(dist_to_obstacle, 2) * obstacle_avoidance_gain;
        float repel_y = (y - obstacle_position_[1]) / pow(dist_to_obstacle, 2) * obstacle_avoidance_gain;
        float repel_z = (z - obstacle_position_[2]) / pow(dist_to_obstacle, 2) * obstacle_avoidance_gain;

        goal_x += repel_x;
        goal_y += repel_y;
        goal_z += repel_z;
    }

    // Normalize and scale the trajectory
    float magnitude = sqrt(goal_x * goal_x + goal_y * goal_y + goal_z * goal_z);
    goal_x = x + (goal_x / magnitude);
    goal_y = y + (goal_y / magnitude);
    goal_z = z + (goal_z / magnitude);
    //std::cout << "goal: " << goal_x << " " << goal_y << " " << goal_z << std::endl;

    return {goal_x, goal_y, goal_z};
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node with obstacle avoidance..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}
