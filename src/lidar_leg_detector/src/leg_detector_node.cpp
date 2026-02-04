#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath> // std::isfinite에 필요 
class LegDetectorNode : public rclcpp::Node {
public:
	LegDetectorNode()
	: Node("leg_detector_node")
	{
		// QoS 프로파일 생성
		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
		qos.best_effort();
		scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",
			qos,
			std::bind(&LegDetectorNode::scanCallback, this, std::placeholders::_1)
		);

		RCLCPP_INFO(this->get_logger(), "Leg detector node start");
	}

private:
	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // 핵심 코드
	{ // 축 기준은 
		const double MIN_ANGLE = -1.047; // -60 deg
		const double MAX_ANGLE = 1.047;
		const double MIN_RANGE = 0.2;
		const double MAX_RANGE = 2.5;
		size_t valid_count = 0;

		for(size_t i = 0; i < msg->ranges.size(); ++i) {
			double angle = msg->angle_min + i * msg->angle_increment;
			double range = msg->ranges[i];

			// 각도 필터 - 전방만 보고 판단
			if (angle < MIN_ANGLE || angle > MAX_ANGLE)
				continue;

			// 거리 필터 - 너무 가깝거나 먼값 제거 
			if (range < MIN_RANGE || range > MAX_RANGE)
				continue;

			// NaN / inf 방지 - 깨진값 제거
			if (!std::isfinite(range))
				continue;

			// 필터에서 안걸러진 부분 = 유효 포인트 
			valid_count++;
		}


		RCLCPP_INFO_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Valid points in FOV: %zu / %zu",
			valid_count,
			msg->ranges.size()
		);
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LegDetectorNode>());
	rclcpp::shutdown();
	return 0;
}
