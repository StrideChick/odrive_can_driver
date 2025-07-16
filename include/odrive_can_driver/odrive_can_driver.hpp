#ifndef ODRIVE_CAN_DRIVER__ODRIVE_CAN_DRIVER_HPP_
#define ODRIVE_CAN_DRIVER__ODRIVE_CAN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <map>

namespace odrive_can_driver
{

// ODrive CAN Commands
enum ODriveCANCommand : uint32_t {
    MSG_GET_VERSION = 0x000,
    MSG_ODRIVE_HEARTBEAT = 0x001,
    MSG_ODRIVE_ESTOP = 0x002,
    MSG_GET_ERROR = 0x003,
    MSG_RX_SDO = 0x004,
    MSG_TX_SDO = 0x005,
    MSG_ADDRESS = 0x006,
    MSG_SET_AXIS_REQUESTED_STATE = 0x007,
    MSG_GET_ENCODER_ESTIMATES = 0x009,
    MSG_SET_CONTROLLER_MODES = 0x00B,
    MSG_SET_INPUT_POS = 0x00C,
    MSG_SET_INPUT_VEL = 0x00D,
    MSG_SET_INPUT_TORQUE = 0x00E,
    MSG_SET_LIMITS = 0x00F,
    MSG_SET_TRAJ_VEL_LIMIT = 0x011,
    MSG_SET_TRAJ_ACCEL_LIMITS = 0x012,
    MSG_SET_TRAJ_INERTIA = 0x013,
    MSG_GET_IQ = 0x014,
    MSG_GET_TEMPERATURE = 0x015,
    MSG_REBOOT = 0x016,
    MSG_GET_BUS_VOLTAGE_CURRENT = 0x017,
    MSG_CLEAR_ERRORS = 0x018,
    MSG_SET_ABSOLUTE_POSITION = 0x019,
    MSG_SET_POS_GAIN = 0x01A,
    MSG_SET_VEL_GAINS = 0x01B,
    MSG_GET_TORQUES = 0x01C,
    MSG_GET_POWERS = 0x01D,
    MSG_ENTER_DFU_MODE = 0x01F
};

// ODrive軸の状態
enum ODriveAxisState : uint32_t {
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_STARTUP_SEQUENCE = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    AXIS_STATE_LOCKIN_SPIN = 9,
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13
};

// コントローラーモード
enum ODriveControlMode : uint32_t {
    CONTROL_MODE_VOLTAGE_CONTROL = 0,
    CONTROL_MODE_TORQUE_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3
};

class MotorStatus {
public:
    MotorStatus();    
    uint32_t getAxisError() const { return axis_error_; }
    void setAxisError(uint32_t error) { axis_error_ = error; }
    
    uint8_t getAxisState() const { return axis_state_; }
    void setAxisState(uint8_t state) { axis_state_ = state; }
    
    uint8_t getProcedureResult() const { return procedure_result_; }
    void setProcedureResult(uint8_t result) { procedure_result_ = result; }
    
    bool isTrajectoryDone() const { return trajectory_done_; }
    void setTrajectoryDone(bool done) { trajectory_done_ = done; }
    
    float getPositionEstimate() const { return position_estimate_; }
    void setPositionEstimate(float position) { 
        position_estimate_ = position; 
        last_encoder_update_ = rclcpp::Clock().now();
    }
    
    float getVelocityEstimate() const { return velocity_estimate_; }
    void setVelocityEstimate(float velocity) { 
        velocity_estimate_ = velocity; 
        last_encoder_update_ = rclcpp::Clock().now();
    }
    
    float getIqSetpoint() const { return iq_setpoint_; }
    void setIqSetpoint(float setpoint) { iq_setpoint_ = setpoint; }
    
    float getIqMeasured() const { return iq_measured_; }
    void setIqMeasured(float measured) { iq_measured_ = measured; }
    
    float getFetTemperature() const { return fet_temperature_; }
    void setFetTemperature(float temp) { fet_temperature_ = temp; }
    
    float getMotorTemperature() const { return motor_temperature_; }
    void setMotorTemperature(float temp) { motor_temperature_ = temp; }
    
    float getBusVoltage() const { return bus_voltage_; }
    void setBusVoltage(float voltage) { bus_voltage_ = voltage; }
    
    float getBusCurrent() const { return bus_current_; }
    void setBusCurrent(float current) { bus_current_ = current; }
    
    float getTorqueTarget() const { return torque_target_; }
    void setTorqueTarget(float target) { torque_target_ = target; }
    
    float getTorqueEstimate() const { return torque_estimate_; }
    void setTorqueEstimate(float estimate) { torque_estimate_ = estimate; }

    float getElectricalPower() const { return electrical_power_; }
    void setElectricalPower(float power) { electrical_power_ = power; }
    
    float getMechanicalPower() const { return mechanical_power_; }
    void setMechanicalPower(float power) { mechanical_power_ = power; }
    
    uint32_t getActiveErrors() const { return active_errors_; }
    void setActiveErrors(uint32_t errors) { active_errors_ = errors; }
    
    uint32_t getDisarmReason() const { return disarm_reason_; }
    void setDisarmReason(uint32_t reason) { disarm_reason_ = reason; }
    
    // タイムスタンプの取得・設定
    rclcpp::Time getLastHeartbeat() const { return last_heartbeat_; }
    void updateHeartbeat() { last_heartbeat_ = rclcpp::Clock().now(); }
    
    rclcpp::Time getLastEncoderUpdate() const { return last_encoder_update_; }
    rclcpp::Time getLastStatusUpdate() const { return last_status_update_; }
    void updateStatusTimestamp() { last_status_update_ = rclcpp::Clock().now(); }
    
    // ヘルパーメソッド
    bool isIdle() const { return axis_state_ == AXIS_STATE_IDLE; }
    bool isClosedLoopControl() const { return axis_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL; }
    bool hasError() const { return axis_error_ != 0 || active_errors_ != 0; }
    bool isHealthy() const { return !hasError() && (isIdle() || isClosedLoopControl()); }

private:
    // ハートビート情報
    uint32_t axis_error_;
    uint8_t axis_state_;
    uint8_t procedure_result_;
    bool trajectory_done_;
    
    // エンコーダ情報
    float position_estimate_;
    float velocity_estimate_;
    
    // 電流情報
    float iq_setpoint_;
    float iq_measured_;
    
    // 温度情報
    float fet_temperature_;
    float motor_temperature_;
    
    // 電源情報
    float bus_voltage_;
    float bus_current_;
    
    // トルク情報
    float torque_target_;
    float torque_estimate_;
    
    // 電力情報
    float electrical_power_;
    float mechanical_power_;
    
    // エラー情報
    uint32_t active_errors_;
    uint32_t disarm_reason_;
    
    // タイムスタンプ
    rclcpp::Time last_heartbeat_;
    rclcpp::Time last_encoder_update_;
    rclcpp::Time last_status_update_;
};

class ODriveCANDriver : public rclcpp::Node
{
public:
    explicit ODriveCANDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ODriveCANDriver();

private:
    // CAN Interface
    int can_socket_ =  -1;
    std::string can_interface_ ;
    std::atomic<bool> running_ = false;
    std::thread can_receive_thread_;
    
    // ODrive Node ID
    uint8_t left_wheel_node_id_;  
    uint8_t right_wheel_node_id_; 
    
    // robot parameters
    double wheel_base_;       // 車輪間距離 [m]
    double wheel_radius_;     // 車輪半径 [m]
    double max_velocity_;     // 最大速度 [m/s]
    double velocity_timeout_; // 速度コマンドタイムアウト [s]
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_axis_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_axis_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_axis_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_axis_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_iq_measured_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_iq_measured_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_fet_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_fet_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_motor_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_motor_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bus_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bus_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_status_ok_pub_;
    
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_odom_time_;
    double current_linear_vel_ = 0.0;
    double current_angular_vel_ = 0.0;
    
    double x_position_ = 0.0;
    double y_position_ = 0.0; 
    double theta_ ;
    double left_wheel_position_ = 0.0; 
    double right_wheel_position_ = 0.0;
    
    // モーター状態
    std::map<uint8_t, MotorStatus> motor_status_;
    
    // 状態監視パラメータ
    double status_publish_rate_;  // 状態パブリッシュ頻度 [Hz]
    double temperature_threshold_; // 温度警告閾値 [°C]
    double voltage_min_threshold_; // 最小電圧閾値 [V]
    double voltage_max_threshold_; // 最大電圧閾値 [V]
    
    // メソッド
    bool init_can_interface();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void differential_drive_kinematics(double linear_vel, double angular_vel, double& left_wheel_vel, double& right_wheel_vel);
    void send_velocity_command(uint8_t node_id, double velocity);
    void send_axis_state_command(uint8_t node_id, ODriveAxisState state);
    void send_controller_mode_command(uint8_t node_id, ODriveControlMode control_mode, uint32_t input_mode);
    void can_receive_loop();
    void process_can_message(const struct can_frame& frame);
    void stop_motors();
    void initialize_odrive(uint8_t node_id);
    
    // オドメトリ関連のメソッド
    void update_odometry();
    void publish_odometry();
    void request_encoder_estimates(uint8_t node_id);
    
    // ユーティリティ
    void float_to_bytes(float value, uint8_t* bytes);
    float bytes_to_float(const uint8_t* bytes);
    void uint32_to_bytes(uint32_t value, uint8_t* bytes);
    uint32_t bytes_to_uint32(const uint8_t* bytes);
};

}  // namespace odrive_can_driver

#endif  // ODRIVE_CAN_DRIVER__ODRIVE_CAN_DRIVER_HPP_
