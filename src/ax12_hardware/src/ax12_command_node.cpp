// ax12_command_node.cpp
// - Read servo positions and publish to /joint_states
// - Subscribe to /joint_states and write goal positions to servos
// - Protocol 1.0, IDs 1~5
// - ID 3 inverted
// - joint limits applied
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <thread>
#include <chrono>

using namespace dynamixel;

// AX-12A Control Table
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 24; // 1 byte
static constexpr uint16_t ADDR_GOAL_POSITION    = 30; // 2 bytes
static constexpr uint16_t ADDR_PRESENT_POSITION = 36; // 2 bytes

static constexpr double AX_TICK_PER_RAD =
  1023.0 / (300.0 * M_PI / 180.0); // ~195.378 tick/rad

class Ax12CommandNode : public rclcpp::Node
{
public:
  Ax12CommandNode() : Node("ax12_command_node"),
    joint_names_({"joint1","joint2","joint3","joint4","joint5"}),
    ids_({1,2,3,4,5}),
    limits_({{
      {-M_PI,  M_PI},        // joint1 (Z)
      {-1.57,  1.57},        // joint2 (Y)
      {-1.57,  1.57},        // joint3 (Y, inverted)
      {-1.57,  1.57},        // joint4 (Y)
      {-M_PI,  M_PI}         // joint5 (Z)
    }})
  {
    RCLCPP_INFO(get_logger(), "Starting initialization...");
    
    device_   = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 1000000);
    publish_rate_ = declare_parameter<double>("publish_hz", 20.0);
    
    RCLCPP_INFO(get_logger(), "Device: %s, Baudrate: %d, Publish rate: %.1f Hz", 
                device_.c_str(), baudrate_, publish_rate_);
    
    port_ = PortHandler::getPortHandler(device_.c_str());
    if (!port_) {
      RCLCPP_ERROR(get_logger(), "Failed to get port handler");
      throw std::runtime_error("getPortHandler failed");
    }
    
    packet_ = PacketHandler::getPacketHandler(1.0);
    if (!packet_) {
      RCLCPP_ERROR(get_logger(), "Failed to get packet handler");
      throw std::runtime_error("getPacketHandler failed");
    }
    
    RCLCPP_INFO(get_logger(), "Opening port...");
    if (!port_->openPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s", device_.c_str());
      throw std::runtime_error("openPort failed");
    }
    
    RCLCPP_INFO(get_logger(), "Setting baudrate...");
    if (!port_->setBaudRate(baudrate_)) {
      RCLCPP_ERROR(get_logger(), "Failed to set baudrate");
      throw std::runtime_error("setBaudRate failed");
    }
    
    // 짧은 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // torque on
    RCLCPP_INFO(get_logger(), "Enabling torque for servos...");
    for (int id : ids_) {
      // 패킷 전송 전 짧은 딜레이
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      
      // 먼저 ping으로 서보 존재 확인
      uint16_t model = 0;
      uint8_t ping_error = 0;
      int ping_result = packet_->ping(port_, id, &model, &ping_error);
      
      if (ping_result != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "ID %d not found, skipping... (%s)", 
                    id, packet_->getTxRxResult(ping_result));
        continue;  // 이 ID는 건너뛰기
      }
      
      RCLCPP_INFO(get_logger(), "Found servo ID %d (model: %d)", id, model);
      
      uint8_t dxl_error = 0;
      int dxl_comm_result = packet_->write1ByteTxRx(
        port_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
      
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "ID %d torque enable comm failed: %s", 
                    id, packet_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_WARN(get_logger(), "ID %d torque enable error: %s", 
                    id, packet_->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(get_logger(), "Torque enabled for ID %d", id);
        active_ids_.push_back(id);  // 성공한 ID만 기록
      }
    }
    
    RCLCPP_INFO(get_logger(), "Active servos: %zu out of %zu", 
                active_ids_.size(), ids_.size());
    
    // Publisher for reading joint states
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // Subscriber for commanding joint positions (optional, for external control)
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", 10,
      std::bind(&Ax12CommandNode::onJointCommand, this, std::placeholders::_1));
    
    // Timer to read and publish joint states
    int period_ms = static_cast<int>(1000.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Ax12CommandNode::publishJointStates, this));
      
    RCLCPP_INFO(get_logger(), "ax12_command_node ready (%s)", device_.c_str());
  }

  ~Ax12CommandNode() override
  {
    // Torque off before closing
    for (int id : active_ids_) {
      uint8_t dxl_error = 0;
      packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    }
    
    if (port_) port_->closePort();
  }

private:
  uint16_t radToTick(int id, double rad) const
  {
    // clamp
    int idx = id - 1;
    if (idx < 0 || idx >= static_cast<int>(limits_.size())) return 512;
    
    rad = std::max(limits_[idx].first, std::min(limits_[idx].second, rad));
    
    // inverted joint (ID 3은 반전 안 함)
    if (id == 3) {
      rad = rad;  // 명시적으로 유지 (삭제하지 말 것!)
    }
    
    int tick = static_cast<int>(std::round(rad * AX_TICK_PER_RAD + 512));
    tick = std::max(0, std::min(1023, tick));
    return static_cast<uint16_t>(tick);
  }
  
  double tickToRad(int id, uint16_t tick) const
  {
    double rad = (static_cast<double>(tick) - 512.0) / AX_TICK_PER_RAD;
    
    // inverted joint (ID 3은 반전 안 함)
    if (id == 3) {
      rad = rad;  // 명시적으로 유지
    }
    
    return rad;
  }
  
  void publishJointStates()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    
    for (int id : active_ids_) {
      size_t idx = id - 1;
      if (idx >= joint_names_.size()) continue;
      
      uint16_t present_pos = 0;
      uint8_t dxl_error = 0;
      int result = packet_->read2ByteTxRx(
        port_, id, ADDR_PRESENT_POSITION, &present_pos, &dxl_error);
      
      if (result == COMM_SUCCESS && dxl_error == 0) {
        double rad = tickToRad(id, present_pos);
        msg.name.push_back(joint_names_[idx]);
        msg.position.push_back(rad);
        msg.velocity.push_back(0.0);  // 속도는 0으로 (필요시 읽을 수 있음)
        msg.effort.push_back(0.0);    // 토크는 0으로 (필요시 읽을 수 있음)
      } else {
        // 에러는 throttle로 출력 (너무 많은 로그 방지)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Read pos failed (ID=%d): result=%d, error=%d", 
                             id, result, dxl_error);
      }
    }
    
    if (!msg.name.empty()) {
      pub_->publish(msg);
    }
  }

  void onJointCommand(const sensor_msgs::msg::JointState &msg)
  {
    // name->position 매핑
    std::unordered_map<std::string,double> m;
    for (size_t i=0; i<msg.name.size() && i<msg.position.size(); ++i)
      m[msg.name[i]] = msg.position[i];
    
    // 활성화된 ID만 처리
    for (int id : active_ids_) {
      size_t idx = id - 1;
      if (idx >= joint_names_.size()) continue;
      
      auto it = m.find(joint_names_[idx]);
      if (it == m.end()) continue;
      
      uint16_t goal = radToTick(id, it->second);
      
      // write2ByteTxOnly 사용 (응답 대기 안 함 - 더 빠르고 안전)
      int dxl_comm_result = packet_->write2ByteTxOnly(
        port_, id, ADDR_GOAL_POSITION, goal);
      
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "ID %d write failed: %s", 
                             id, packet_->getTxRxResult(dxl_comm_result));
      }
    }
  }

private:
  std::string device_;
  int baudrate_;
  double publish_rate_;
  std::array<std::string,5> joint_names_;
  std::array<int,5> ids_;
  std::vector<int> active_ids_;
  std::array<std::pair<double,double>,5> limits_;
  PortHandler *port_{nullptr};
  PacketHandler *packet_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ax12CommandNode>());
  rclcpp::shutdown();
  return 0;
}
