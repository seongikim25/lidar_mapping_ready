#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

using namespace dynamixel;

#define DEVICENAME "/dev/ttyUSB1"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 1.0

#define AX_ID 5

// AX-12A Control Table
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  PortHandler *portHandler = PortHandler::getPortHandler(DEVICENAME);
  PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    printf("Failed to open port\n");
    return 1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    printf("Failed to set baudrate\n");
    return 1;
  }

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // Torque ON
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, AX_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    printf("Torque enable failed\n");
    return 1;
  }

  // Move
  packetHandler->write2ByteTxRx(
    portHandler, AX_ID, ADDR_GOAL_POSITION, 700, &dxl_error);

  rclcpp::sleep_for(std::chrono::seconds(2));

  // Back to center
  packetHandler->write2ByteTxRx(
    portHandler, AX_ID, ADDR_GOAL_POSITION, 512, &dxl_error);

  portHandler->closePort();
  rclcpp::shutdown();
  return 0;
}
