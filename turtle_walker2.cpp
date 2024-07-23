#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

using namespace std::chrono_literals;

class SpawnTurtleClient : public rclcpp::Node
{
public:
    SpawnTurtleClient(const std::string& name)
    : Node(name)
    {
        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 5.0;
        request->y = 5.0;
        request->theta = 0.0;
        request->name = "Tom_XXXX";

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto future = client_->async_send_request(request);
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Spawned new turtle: %s", response->name.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto turtle_walker2 = std::make_shared<SpawnTurtleClient>("turtle_walker2");
    rclcpp::spin(turtle_walker2);
    rclcpp::shutdown();
    return 0;
}