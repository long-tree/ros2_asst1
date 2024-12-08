#include"rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include<string>
#include"geometry_msgs/msg/twist.hpp"
#include"turtlesim/msg/pose.hpp"
#include <rclcpp/node.hpp>
#include <turtlesim/msg/detail/pose__struct.hpp>
class TurtleFollow : public rclcpp::Node
{
   public:
   TurtleFollow(const std::string name) :Node("follow"+name)
   {
     publisher_=this->create_publisher<geometry_msgs::msg::Twist>(name+"/cmd_vel", 10);
     subscriber_=this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleFollow::on_pose_received,this,std::placeholders::_1));
     sign_subscriber_=this->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10, 
                                                                    [&](geometry_msgs::msg::Twist twist)
                                                                    {
                                                                        v_sign_=twist.linear.x>0?1:-1;
                                                                    });
   }
   private:
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sign_subscriber_;
   int v_sign_;
   private:
   void on_pose_received(turtlesim::msg::Pose pose)
   {
     auto msg=geometry_msgs::msg::Twist();
     msg.angular.z=pose.angular_velocity;
     msg.linear.x=pose.linear_velocity*v_sign_;
     publisher_->publish(msg);
   }
};
int main(int argc,char** argv)
{
    if(argc<=2)
    {
      RCLCPP_INFO(rclcpp::get_logger("follow"),"usage: follow count follow_turtle1.......");
      return 1;
    }
    rclcpp::init(argc,argv);
        std::shared_ptr<TurtleFollow> nodearr[20];
      rclcpp::executors::MultiThreadedExecutor executor;
    int n=2;
    for(int i=0;i<n;i++)
    {
        nodearr[i]=std::make_shared<TurtleFollow>(argv[i+2]);
        executor.add_node(nodearr[i]);
    }
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    executor_thread.join();
    rclcpp::shutdown();
    return 0;


}
