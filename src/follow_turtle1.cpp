#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <memory>

class node: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target,catcher;
    bool reach_flag;

    // Well, these two variables are not used in the code, but they may help in the task
    bool inital_flag;
    geometry_msgs::msg::TransformStamped tf_inital;
  
    void timer_callback()
    {

        geometry_msgs::msg::TransformStamped tf;     
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);

        } catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        // now the code was implemented to make the catcher follow the target
        // you should modify it to keep the relative position and orientation between the target and the catcher
        // you can first try to make the catcher keep a fixed distance from the target
        // then you can try to make the catcher keep a fixed angle from the target
        // these two tasks are not easy, but you can do it!
         
        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();

        if (hypot(t.translation.x, t.translation.y) < 0.1)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            if (!reach_flag) RCLCPP_INFO(this->get_logger(), "Reached target");
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        message.linear.x = 0.5 * hypot(t.translation.x, t.translation.y);
        message.angular.z = 1.0 * atan2(t.translation.y, t.translation.x);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);

    }
public: 
    node(std::string target, std::string catcher): Node("follower"), target(target), catcher(catcher)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher+"/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Hello, world");   
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
};

int main(int argc, char *argv[])
{
    if (argc != 4 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher1 catcher2");
        return 1;
    }
    rclcpp::init(argc,argv);
    std::shared_ptr<node> nodearr[2];
      rclcpp::executors::MultiThreadedExecutor executor;
    int n=2;
    for(int i=0;i<n;i++)
    {
        nodearr[i]=std::make_shared<node>(argv[1],argv[i+2]);
        executor.add_node(nodearr[i]);
    }
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}