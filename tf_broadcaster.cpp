#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"


class TurtleTfBroadcaster : public rclcpp::Node
{
public:
    TurtleTfBroadcaster(const std::string &name) : Node(name)
    { 
        this->declare_parameter("turtle","turtle1");
        turtle=this->get_parameter("turtle").as_string();
        // 创建一个动态广播器
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 创建一个乌龟位姿订阅方
        // pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        //      "/" + turtle + "/pose",10,
        //     [this](const turtlesim::msg::Pose::SharedPtr msg) {
        //         do_pose(*msg);
        //     });
        pose_sub_=this->create_subscription<turtlesim::msg::Pose>(
            "/"+turtle+"/pose",10,std::bind(&TurtleTfBroadcaster::do_pose,this, std::placeholders::_1));
    
    }

private:
    
    // 回调函数中，获取乌龟位姿并生成相对关系并发布
    void do_pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        // 组织消息
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = turtle;

        ts.transform.translation.x = pose->x;
        ts.transform.translation.y = pose->y;
        ts.transform.translation.z = 0.0;

        // 从欧拉角转换出四元数
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, pose->theta); // 将角度转换为弧度
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        // 发布
        broadcaster_->sendTransform(ts);
    }
    std::string turtle;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    //rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
}; 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto tf_broadcaster = std::make_shared<TurtleTfBroadcaster>("tf_broadcaster");
    rclcpp::spin(tf_broadcaster);
    rclcpp::shutdown();
    return 0;
}
