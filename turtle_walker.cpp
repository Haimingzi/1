#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <iostream>

class TurtleFactory : public rclcpp::Node {
public:
    TurtleFactory(const std::string& name) : Node(name) {
        // 设置参数并初始化小海龟的坐标和名字
        this->declare_parameter("x",4.0);
        this->declare_parameter("y",4.0);
        this->declare_parameter("theta",0.0);
        this->declare_parameter("turtle_name","Tom_XXXX");
        x = this->get_parameter("x").as_double();
        y = this->get_parameter("y").as_double();
        theta = this->get_parameter("theta").as_double();
        turtle_name = this->get_parameter("turtle_name").as_string();
        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    }

     void create_turtle() {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y; // 确保设置 y 坐标
        request->theta = theta;
        request->name = turtle_name;
        auto future = client_->async_send_request(request);
    }
private:
    double x, y, theta;
    std::string turtle_name;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto turtle_walker = std::make_shared<TurtleFactory>("turtle_walker");
    turtle_walker->create_turtle(); // 调用 create_turtle 方法执行创建海龟操作
    rclcpp::spin(turtle_walker);
    rclcpp::shutdown();
    return 0;
}
