#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>

class LidarPreprocessNode : public rclcpp::Node
{
public:
    LidarPreprocessNode()
    : Node("lidar_preprocess_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarPreprocessNode::scanCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/lidar_target",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Lidar preprocess node started");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const double FRONT_RAD = 0.523;   // ±30 deg
        const double MIN_DIST  = 0.2;
        const double MAX_DIST  = 3.0;

        double min_dist = 1e6;
        double sum_angle = 0.0;
        int count = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double r = msg->ranges[i];
            if (!std::isfinite(r)) continue;
            if (r < MIN_DIST || r > MAX_DIST) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            if (angle < -FRONT_RAD || angle > FRONT_RAD) continue;

            if (r < min_dist) min_dist = r;
            sum_angle += angle;
            count++;
        }

        std_msgs::msg::Float32MultiArray out;
        out.data.clear();

        if (count >= 5)
        {
            out.data.push_back(1.0f);                     // detected
            out.data.push_back(static_cast<float>(min_dist));
            out.data.push_back(static_cast<float>(sum_angle / count));
        }
        else
        {
            out.data.push_back(0.0f);
            out.data.push_back(0.0f);
            out.data.push_back(0.0f);
        }

        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPreprocessNode>());
    rclcpp::shutdown();
    return 0;
}

