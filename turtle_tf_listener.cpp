#include"rclcpp/rclcpp.hpp"
#include"tf2_ros/buffer.h"
#include"tf2_ros/transform_listener.h"
#include"geometry_msgs/msg/twist.hpp" 
using namespace std::chrono_literals;

class TfListener: public rclcpp::Node{

public:

TfListener(const std::string&name):Node(name)
{   //声明参数服务
    this->declare_parameter("father_frame","Tom_XXXX");
    this->declare_parameter("child_frame","turtle1");
    father_frame=this->get_parameter("father_frame").as_string();
    child_frame=this->get_parameter("child_frame").as_string();
    //  创建tf缓存对象指针，融合多个坐标系相对关系为一颗坐标树
    buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    //  创建tf监听器，绑定缓存对象，会将所有广播器广播的数据写入缓存
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    //  编写一个定时器，循环实现转换
    cmd_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/"+father_frame+"/cmd_vel",10);
    timer_=this->create_wall_timer(1s,std::bind(&TfListener::on_timer,this));

}

private:

void on_timer()
{
//实现坐标系转换

try
{
   //实现坐标变换
auto ts = buffer_->lookupTransform(father_frame,child_frame,tf2::TimePointZero);
   //组织并发布速度指令
geometry_msgs::msg::Twist twist;
twist.linear.x=0.8*sqrt(pow(ts.transform.translation.x,2) + pow(ts.transform.translation.y,2));
twist.angular.z=1.0*atan2(ts.transform.translation.y,ts.transform.translation.x);

cmd_pub_->publish(twist);
}
catch(const tf2::LookupException& e)
{
//std::cout<<e.what()<<'\n';
RCLCPP_INFO(this->get_logger(),"异常提示：%s",e.what());
}

}
std::string father_frame;
std::string child_frame;
std::shared_ptr<tf2_ros::Buffer>buffer_;
std::shared_ptr<tf2_ros::TransformListener>listener_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const* argv[])
{
//2.初始化ROS2客户端
rclcpp::init(argc,argv);
//4.调用spin函数,并传入节点对象指针
//auto tf_listener1=std::make_shared<TFListener>("tf_listener1");
auto turtle_tf_listener = std::make_shared<TfListener>("turtle_tf_listener");
rclcpp::spin(turtle_tf_listener);
//5.资源释放
rclcpp::shutdown();
return 0;
}

