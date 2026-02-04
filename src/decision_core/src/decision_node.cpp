#include "rclcpp/rclcpp.hpp"
#include "decision_interfaces/msg/decision.hpp"
#include "ontology.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("decision_node")
  {
    auto share_dir =
      ament_index_cpp::get_package_share_directory("decision_core");
    std::string ontology_path = share_dir + "/knowledge/ontology.yaml";

    if (ontology_.load(ontology_path)) {
      RCLCPP_INFO(get_logger(), "✅ Ontology loaded successfully");
    }

    const auto* item = ontology_.get("knife");
    if (item) {
      RCLCPP_INFO(
        get_logger(),
        "knife: dangerous=%d, fragile=%d",
        item->dangerous,
        item->fragile
      );
    }
    
    // const auto *item = ontology_.get("apple");
    // if (item) {
    //   RCLCPP_INFO(
    //   	get_logger(),
    //   	"apple: dangerous=%d, fragile=%d",
    //   	item->dangerous,
    //   	item->fragile
    //   );
    // }
    
    // const auto *item = ontology_.get("glass");
    // if (item) {
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "glass: dangerous=%d, fragile=%d",
    //     item->dangerous,
    //     item->fragile
    //   );
    // }

    // const auto *item = ontology_.get("egg");
    // if (item) {
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "egg: dangerous=%d, fragile=%d",
    //     item->dangerous,
    //     item->fragile
    //   );
    // }
    

    pub_ = create_publisher<decision_interfaces::msg::Decision>(
      "/decision", 10
    );
  }

private:
  Ontology ontology_;
  rclcpp::Publisher<decision_interfaces::msg::Decision>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
  return 0;
}

