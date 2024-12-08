#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include<cmath>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <turtlesim/msg/detail/pose__struct.hpp>
class FollowNode : public rclcpp::Node
{
   public:
   FollowNode() : Node("follow_node")
   {
     publisher_=this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
     pose_subscriber_=this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&FollowNode::on_receive_pose,this,std::placeholders::_1));
     now_subscriber_=this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, [&](turtlesim::msg::Pose now)
                                                                                                          {
                                                                                                            now_x_=now.x;
                                                                                                            now_y_=now.y;
                                                                                                            now_angle_=now.theta;
                                                                                                          }  );
   }
   private:
   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr now_subscriber_;
   double now_x_,now_y_,now_angle_;
   private:
   void on_receive_pose(const turtlesim::msg::Pose pose)
   {
    auto msg=geometry_msgs::msg::Twist();
    double target_x=pose.x;
    double target_y=pose.y;
    double error_distance=sqrt((target_x-now_x_)*(target_x-now_x_)+(target_y-now_y_)*(target_y-now_y_));
    double error_angle=atan2(target_y-now_y_,target_x-now_x_)-now_angle_;
    double k_p=1;
    if(error_distance>0.1)
    {
         if(fabs(error_angle)>0.2)
         {msg.angular.z=k_p*error_angle;}
         else msg.linear.x=k_p*error_distance;
    }
    publisher_->publish(msg);
   }
};
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<FollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}