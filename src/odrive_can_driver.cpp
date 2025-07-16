#include "odrive_can_driver/odrive_can_driver.hpp"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace odrive_can_driver
{

// MotorStatusクラスの実装
MotorStatus::MotorStatus() : 
    axis_error_(0), axis_state_(0), procedure_result_(0), trajectory_done_(false),
    position_estimate_(0.0f), velocity_estimate_(0.0f),
    iq_setpoint_(0.0f), iq_measured_(0.0f),
    fet_temperature_(0.0f), motor_temperature_(0.0f),
    bus_voltage_(0.0f), bus_current_(0.0f),
    torque_target_(0.0f), torque_estimate_(0.0f),
    electrical_power_(0.0f), mechanical_power_(0.0f),
    active_errors_(0), disarm_reason_(0)
{
    rclcpp::Clock clock;
    last_heartbeat_ = clock.now();
    last_encoder_update_ = clock.now();
    last_status_update_ = clock.now();
}

ODriveCANDriver::ODriveCANDriver(const rclcpp::NodeOptions & options)
: Node("odrive_can_driver", options)
{
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("left_wheel_node_id", 1);
    this->declare_parameter("right_wheel_node_id", 2);
    this->declare_parameter("wheel_base", 0.3);      
    this->declare_parameter("wheel_radius", 0.05);   
    this->declare_parameter("max_velocity", 1.0);    
    this->declare_parameter("velocity_timeout", 1.0); 
    
    can_interface_ = this->get_parameter("can_interface").as_string();
    left_wheel_node_id_ = this->get_parameter("left_wheel_node_id").as_int();
    right_wheel_node_id_ = this->get_parameter("right_wheel_node_id").as_int();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    velocity_timeout_ = this->get_parameter("velocity_timeout").as_double();
    
    // CAN interface Initialization
    if (!init_can_interface()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
        return;
    }
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ODriveCANDriver::cmd_vel_callback, this, _1));
    
    left_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("left_wheel_velocity", 10);
    right_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("right_wheel_velocity", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    auto odom_timer = this->create_wall_timer(50ms, std::bind(&ODriveCANDriver::update_odometry, this));
    
    // ODrive Info Initialization
    initialize_odrive(left_wheel_node_id_);
    initialize_odrive(right_wheel_node_id_);
    
    
    can_receive_thread_ = std::thread(&ODriveCANDriver::can_receive_loop, this);
    
    last_cmd_time_ = this->now();
    last_odom_time_ = this->now();
}

ODriveCANDriver::~ODriveCANDriver()
{
    running_ = false;
    stop_motors();
    if (can_receive_thread_.joinable()) {
        can_receive_thread_.join();
    }    
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
    RCLCPP_INFO(this->get_logger(), "ODrive CAN Driver shutdown");
}

bool ODriveCANDriver::init_can_interface()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return false;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get CAN interface index for %s", can_interface_.c_str());
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    running_ = true;
    RCLCPP_INFO(this->get_logger(), "CAN interface %s initialized successfully", can_interface_.c_str());
    return true;
}

void ODriveCANDriver::initialize_odrive(uint8_t node_id)
{
    struct can_frame frame;
    frame.can_id = (node_id << 5) | MSG_CLEAR_ERRORS;
    frame.can_dlc = 0;
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send clear errors command to node %d", node_id);
        return;
    }
    std::this_thread::sleep_for(100ms);
    send_controller_mode_command(node_id, CONTROL_MODE_VELOCITY_CONTROL, 1); // input_mode = 1 (vel_ramp)
    std::this_thread::sleep_for(100ms); 
    send_axis_state_command(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void ODriveCANDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_time_ = this->now();
    double linear_vel = std::clamp(msg->linear.x, -max_velocity_, max_velocity_);
    double angular_vel = msg->angular.z;
    
    current_linear_vel_ = linear_vel;
    current_angular_vel_ = angular_vel;
    
    // 差動駆動の運動学計算
    double left_wheel_vel, right_wheel_vel;
    differential_drive_kinematics(linear_vel, angular_vel, left_wheel_vel, right_wheel_vel);
    
    // 車輪速度を回転速度[turns/s]に変換 (ODriveは turns/s で制御)
    double left_turns_per_sec = left_wheel_vel / (2.0 * M_PI * wheel_radius_);
    double right_turns_per_sec = right_wheel_vel / (2.0 * M_PI * wheel_radius_);
    
    // ODriveに速度コマンドを送信
    send_velocity_command(left_wheel_node_id_, left_turns_per_sec);
    send_velocity_command(right_wheel_node_id_, right_turns_per_sec);
    
    // ROS2トピックに公開
    auto left_msg = std::make_unique<std_msgs::msg::Float32>();
    auto right_msg = std::make_unique<std_msgs::msg::Float32>();
    left_msg->data = left_turns_per_sec;
    right_msg->data = right_turns_per_sec;
    
    left_velocity_pub_->publish(std::move(left_msg));
    right_velocity_pub_->publish(std::move(right_msg));
}

void ODriveCANDriver::differential_drive_kinematics(double linear_vel, double angular_vel, 
                                                   double& left_wheel_vel, double& right_wheel_vel)
{
    // 差動駆動の運動学
    // v_left = v - (w * L) / 2
    // v_right = v + (w * L) / 2
    // v = 線形速度、w = 角速度、L = 車輪間距離
    
    left_wheel_vel = linear_vel - (angular_vel * wheel_base_) / 2.0;
    right_wheel_vel = linear_vel + (angular_vel * wheel_base_) / 2.0;
}

void ODriveCANDriver::send_velocity_command(uint8_t node_id, double velocity)
{
    struct can_frame frame;
    frame.can_id = (node_id << 5) | MSG_SET_INPUT_VEL;
    frame.can_dlc = 8;
    
    // 速度をfloatとしてバイト配列に変換
    float vel_float = static_cast<float>(velocity);
    float ff_torque = 0.0f; // フィードフォワードトルク
    float_to_bytes(vel_float, &frame.data[0]);
    float_to_bytes(ff_torque, &frame.data[4]);
    
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send velocity command to node %d", node_id);
    }
}

void ODriveCANDriver::stop_motors()
{
    send_velocity_command(left_wheel_node_id_, 0.0);
    send_velocity_command(right_wheel_node_id_, 0.0);
}

void ODriveCANDriver::send_controller_mode_command(uint8_t node_id, ODriveControlMode control_mode, uint32_t input_mode)
{
    struct can_frame frame;
    frame.can_id = (node_id << 5) | MSG_SET_CONTROLLER_MODES;
    frame.can_dlc = 8;
    
    uint32_to_bytes(static_cast<uint32_t>(control_mode), &frame.data[0]);
    uint32_to_bytes(input_mode, &frame.data[4]);
    
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send controller mode command to node %d", node_id);
    }
}

void ODriveCANDriver::send_axis_state_command(uint8_t node_id, ODriveAxisState state)
{
    struct can_frame frame;
    frame.can_id = (node_id << 5) | MSG_SET_AXIS_REQUESTED_STATE;
    frame.can_dlc = 4;
    
    uint32_to_bytes(static_cast<uint32_t>(state), frame.data);
    
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send axis state command to node %d", node_id);
    }
}




void ODriveCANDriver::can_receive_loop()
{
    struct can_frame frame;
    while (running_) {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (running_) {
                RCLCPP_ERROR(this->get_logger(), "CAN read error");
            }
            break;
        }
        if (nbytes == sizeof(frame)) {
            process_can_message(frame);
        }
    }
}

void ODriveCANDriver::process_can_message(const struct can_frame& frame)
{
    uint8_t node_id = (frame.can_id >> 5) & 0x3F;  // 6ビット (ビット10-5)
    uint32_t cmd_id = frame.can_id & 0x1F;          // 5ビット (ビット4-0)
    
    switch (cmd_id) {
        case MSG_ODRIVE_HEARTBEAT:
            // RCLCPP_DEBUG(this->get_logger(), "Heartbeat from node %d", node_id);
            break;
            
        case MSG_GET_ENCODER_ESTIMATES:
            if (frame.can_dlc >= 8) {
                float pos = bytes_to_float(&frame.data[0]);
                float vel = bytes_to_float(&frame.data[4]);
                
                // モーター状態を更新
                motor_status_[node_id].setPositionEstimate(pos);
                motor_status_[node_id].setVelocityEstimate(vel);
                
                // RCLCPP_DEBUG(this->get_logger(), "Node %d - Position: %.3f, Velocity: %.3f", node_id, pos, vel);
            }
            break;
            
        default:
            // その他のメッセージ
            break;
    }
}


void ODriveCANDriver::request_encoder_estimates(uint8_t node_id)
{
    struct can_frame frame;
    frame.can_id = (node_id << 5) | MSG_GET_ENCODER_ESTIMATES;
    frame.can_dlc = 0;  // リクエストなのでデータ長は0
    
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send encoder estimates request to node %d", node_id);
    }
}

void ODriveCANDriver::update_odometry()
{
    // エンコーダ値をリクエスト
    request_encoder_estimates(left_wheel_node_id_);
    request_encoder_estimates(right_wheel_node_id_);
    
    // モーター状態を確認
    auto left_status = motor_status_.find(left_wheel_node_id_);
    auto right_status = motor_status_.find(right_wheel_node_id_);
    
    if (left_status == motor_status_.end() || right_status == motor_status_.end()) {
        return;
    }
    
    // 現在の時刻を取得
    auto current_time = this->now();
    double dt = (current_time - last_odom_time_).seconds();
    
    if (dt <= 0.0) {
        return; 
    }
    
    double prev_left_pos = left_wheel_position_;
    double prev_right_pos = right_wheel_position_;    

    left_wheel_position_ = left_status->second.getPositionEstimate();
    right_wheel_position_ = right_status->second.getPositionEstimate();
    
    double delta_left = left_wheel_position_ - prev_left_pos;
    double delta_right = right_wheel_position_ - prev_right_pos;    
    double left_distance = delta_left * wheel_radius_;
    double right_distance = delta_right * wheel_radius_;
    
    double delta_distance = (left_distance + right_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / wheel_base_;
    
    double delta_x = delta_distance * cos(theta_ + delta_theta / 2.0);
    double delta_y = delta_distance * sin(theta_ + delta_theta / 2.0);
    
    x_position_ += delta_x;
    y_position_ += delta_y;
    theta_ += delta_theta;
    
    // 角度を正規化 (-π to π)
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
    double linear_velocity = delta_distance / dt;
    double angular_velocity = delta_theta / dt;
    
    current_linear_vel_ = linear_velocity;
    current_angular_vel_ = angular_velocity;
    
    publish_odometry();    
    last_odom_time_ = current_time;
}

void ODriveCANDriver::publish_odometry()
{
    auto current_time = this->now();
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_position_;
    odom_msg.pose.pose.position.y = y_position_;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odom_msg.twist.twist.linear.x = current_linear_vel_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = current_angular_vel_;
    
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[35] = 0.1;  // yaw
    odom_msg.twist.covariance[0] = 0.1;  // linear.x
    odom_msg.twist.covariance[35] = 0.1; // angular.z
    
    odom_pub_->publish(odom_msg);
    
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    
    transform.transform.translation.x = x_position_;
    transform.transform.translation.y = y_position_;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
}

// ユーティリティ関数
//#############################################################################
void ODriveCANDriver::float_to_bytes(float value, uint8_t* bytes)
{
    memcpy(bytes, &value, sizeof(float));
}

float ODriveCANDriver::bytes_to_float(const uint8_t* bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(float));
    return value;
}

void ODriveCANDriver::uint32_to_bytes(uint32_t value, uint8_t* bytes)
{
    memcpy(bytes, &value, sizeof(uint32_t));
}

uint32_t ODriveCANDriver::bytes_to_uint32(const uint8_t* bytes)
{
    uint32_t value;
    memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}
//#############################################################################


}  // namespace odrive_can_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(odrive_can_driver::ODriveCANDriver)
