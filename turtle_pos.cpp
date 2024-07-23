#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtlePositionSubscriber : public rclcpp::Node {
public:
    TurtlePositionSubscriber() : Node("turtle_position_subscriber") {
        // 创建订阅器
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "Jerry_XXXX/pose", 10, std::bind(&TurtlePositionSubscriber::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Position subscriber %s is ready.", get_name());
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received position: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(50000)); 
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto turtle_pos_sub = std::make_shared<TurtlePositionSubscriber>();
    rclcpp::spin(turtle_pos_sub);
    rclcpp::shutdown();
    return 0;
}