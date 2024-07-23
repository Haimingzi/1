#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h> // 引入具体 Quaternion 类的头文件

using namespace tf2; // 显式地使用 tf2 命名空间

class TurtleTfBroadcaster : public rclcpp::Node
{
public:
    TurtleTfBroadcaster(const std::string& name)
    : Node(name)
    {
        // 创建一个 TransformBroadcaster 对象
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅每个海龟的 pose 题目
        tom_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/Tom_XXXX/pose", 10, std::bind(&TurtleTfBroadcaster::handle_tom_pose, this, std::placeholders::_1));

        jerry_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/Jerry_XXXX/pose", 10, std::bind(&TurtleTfBroadcaster::handle_jerry_pose, this, std::placeholders::_1));
    }

private:
    void handle_tom_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        publish_transform("world", "Tom_XXXX", msg);
    }

    void handle_jerry_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        publish_transform("world", "Jerry_XXXX", msg);
    }

    void publish_transform(const std::string& parent_frame, const std::string& child_frame, const turtlesim::msg::Pose::SharedPtr pose)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = parent_frame;
        transform_stamped.child_frame_id = child_frame;

        transform_stamped.transform.translation.x = pose->x;
        transform_stamped.transform.translation.y = pose->y;
        transform_stamped.transform.translation.z = 0.0;

        Quaternion q;
        q.setRPY(0, 0, pose->theta);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr tom_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr jerry_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto tf_broadcaster = std::make_shared<TurtleTfBroadcaster>("tf_broadcaster");
    rclcpp::spin(tf_broadcaster);
    rclcpp::shutdown();
    return 0;
}