#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class FollowTurtle : public rclcpp::Node
{
public:
    FollowTurtle(const std::string& name)
    : Node(name),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/Tom_XXXX/cmd_vel", 10);

        // Create a timer to call the timer_callback function at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Timer period
            std::bind(&FollowTurtle::timer_callback, this)  // Timer callback function
        );
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("Tom_XXXX","Jerry_XXXX", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform Tom_XXXX to Jerry_XXXX: %s", ex.what());
            return;
        }

        double dx = transform_stamped.transform.translation.x;
        double dy = transform_stamped.transform.translation.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        auto cmd = geometry_msgs::msg::Twist();
        if (distance > 0.1) {
            cmd.linear.x = 1.0 * distance;
            cmd.angular.z = 4.0 * std::atan2(dy, dx);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto follow_turtle = std::make_shared<FollowTurtle>("follow_turtle");
    rclcpp::spin(follow_turtle);
    rclcpp::shutdown();
    return 0;
}