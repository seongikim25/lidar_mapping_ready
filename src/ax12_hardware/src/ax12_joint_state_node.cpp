#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

using namespace dynamixel;

constexpr char DEVICENAME[] = "/dev/ttyUSB0";
constexpr int BAUDRATE = 1000000;
constexpr double AX_RAD_PER_TICK =
    300.0 * M_PI / 180.0 / 1023.0;

// AX-12A Control Table
constexpr uint16_t ADDR_PRESENT_POSITION = 36;

// ID = joint 번호
int ids[5] = {1, 2, 3, 4, 5};

class Ax12JointStateNode : public rclcpp::Node
{
public:
  Ax12JointStateNode()
  : Node("ax12_joint_state_node")
  {
    portHandler_ = PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = PacketHandler::getPacketHandler(1.0);

    portHandler_->openPort();
    portHandler_->setBaudRate(BAUDRATE);

    joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Ax12JointStateNode::timerCallback, this));
  }

private:
  double encoderToRad(int id, int pos)
  {
    double rad = (pos - 512) * AX_RAD_PER_TICK;
    if (id == 3) rad = -rad;  // ★ elbow 반전
    return rad;
  }

  void timerCallback()
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();

    joint_state.name = {
      "joint1", "joint2", "joint3", "joint4", "joint5"
    };

    std::vector<double> positions;

    for (int id : ids) {
      uint16_t pos = 0;
      uint8_t error = 0;
      packetHandler_->read2ByteTxRx(
        portHandler_, id, ADDR_PRESENT_POSITION, &pos, &error);

      positions.push_back(encoderToRad(id, pos));
    }

    joint_state.position = positions;
    joint_pub_->publish(joint_state);
  }

  // members
  PortHandler *portHandler_;
  PacketHandler *packetHandler_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ax12JointStateNode>());
  rclcpp::shutdown();
  return 0;
}

