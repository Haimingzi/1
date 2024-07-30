#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include "turtlesim/msg/pose.hpp"
#include <thread>
#include <chrono>

class TurtleCircleController : public rclcpp::Node {
public:
    TurtleCircleController(const std::string& name) : Node(name) {
        // 创建服务客户端
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        client_->wait_for_service(std::chrono::seconds(1));
        RCLCPP_INFO(this->get_logger(), "Service 'turtle1/teleport_absolute' is available.");

        // 创建发布器用于发布位置信息
        pose_publisher_ = this->create_publisher<turtlesim::msg::Pose>("Jerry_XXXX/pose", 10);
        auto pose_msg = std::make_shared<turtlesim::msg::Pose>();
        
        // 初始位置
        pose_msg->x = 5.5;
        pose_msg->y = 5.5;
        pose_msg->theta = 0.0; // 朝向

        // 发布初始位置
        pose_publisher_->publish(*pose_msg);

        // 创建一个线程用于循环移动和更新位置
        std::thread move_thread(&TurtleCircleController::spinCircle, this);
        move_thread.detach();
    }

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_publisher_;
    
    void spinCircle() {
        const double radius = 3.0;
        const double angle_step =-0.01; // 减小角度步长以减慢角速度
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) {
            for (double angle = 0.0; angle < 2 * M_PI; angle += angle_step) {
                double x = 5.5+radius * cos(angle);
                double y = 5.5+radius * sin(angle);
                
                // 发送移动请求
                auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
                request->x = x;
                request->y = y;
                request->theta = 0.0; // 保持朝向

                RCLCPP_INFO(this->get_logger(), "Moving to (%f, %f)", x, y);
                client_->async_send_request(request);
                
                // 更新位置消息并发布
                auto pose_msg = std::make_shared<turtlesim::msg::Pose>();
                pose_msg->x = x;
                pose_msg->y = y;
                pose_msg->theta = 0.0; // 保持朝向
                pose_publisher_->publish(*pose_msg);

                rate.sleep();  
            }
            if (!rclcpp::ok()) {
                // 正确地退出循环和节点
                break;
            }
        }
    }
 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<TurtleCircleController>("turtlesim_circle_controller");
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
