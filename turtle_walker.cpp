#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include "turtlesim/msg/pose.hpp"

#include <sstream>
#include <iostream>

class TurtleFactory : public rclcpp::Node {
public:
    TurtleFactory(const std::string& name) : Node(name) {
        create_turtle("Jerry_XXXX"); // 使用默认构造函数创建小海龟的名称
    }
private:
    std::string turtle_name_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;

    void create_turtle(const std::string& name) {
        turtle_name_ = name;
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = 5.5;
        request->y = 5.5;
        request->theta =0;
        auto future = client_->async_send_request(request);

         
    }
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto turtle_walker = std::make_shared<TurtleFactory>("turtle_walker");
    rclcpp::spin(turtle_walker);
    rclcpp::shutdown();
    return 0;
}