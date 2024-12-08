#include "rclcpp/rclcpp.hpp"
#include"iostream"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include <rclcpp/utilities.hpp>
#include <string>
#include<thread>

class node: public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tfb;
    std::string turtleName;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtleName;
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tfb.sendTransform(t);
    }
public:
    node(std::string st): Node(st + "_boardcaster"), tfb(this), turtleName(st)
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            turtleName+"/pose", 10, std::bind(&node::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Hello, world");
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
    
};


int main(int argc, char *argv[])
{
    if (argc != 2 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: tf turtle_count");
        return 1;
    }
    rclcpp::init(argc,argv);
    int n=atoi(argv[1]);
    std::string turtle_name[10];
    for(int i=0;i<n;i++)
    {
    turtle_name[i]="turtle"+std::to_string(i+1);
    }
    
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<node> nodearr[10];
    for(int i=0;i<n;i++)
    {
        nodearr[i]=std::make_shared<node>(turtle_name[i]);
        executor.add_node(nodearr[i]);
    }
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}