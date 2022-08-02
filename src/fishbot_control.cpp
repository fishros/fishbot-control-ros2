#include "fishbot/driver/fishbot_driver.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>

using namespace fishbot::driver; // NOLINT
using namespace fish_protocol;   // NOLINT
using namespace boost::placeholders;

class FishBotControl : public rclcpp::Node
{
private:
  FishBotConfig fishbot_config_;
  std::shared_ptr<FishBotDriver> fishbot_driver_ptr_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

private:
  void _initParams();
  void _odomDataCallback(const fishbot_odom_t &odom,
                         const fishbot_speed_t &speed);
  void _cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr &msg);

public:
  FishBotControl(/* args */);
  ~FishBotControl();
};

void FishBotControl::_initParams()
{
  ProtocolConfig proto_config;
  proto_config.protocol_type_ = PROTOCOL_TYPE::UDP_SERVER;
  proto_config.udp_server_ip_ = "0.0.0.0";
  proto_config.udp_server_port_ = 3474;
  fishbot_config_.protocol_config_ = proto_config;

  MotionModelConfig motion_model_config;
  motion_model_config.model_name = "diff2";
  motion_model_config.diff2_distance = 0.170;
  motion_model_config.diff2_pulse = 3293;
  motion_model_config.diff2_radius = 0.065 / 2;
  fishbot_config_.motion_model_config_ = motion_model_config;
}

FishBotControl::FishBotControl(/* args */) : rclcpp::Node("fishbot_control")
{
  _initParams();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub_ptr_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  fishbot_driver_ptr_ = std::make_shared<FishBotDriver>(fishbot_config_);
  // register odom callback
  fishbot_driver_ptr_->SetOdomCallback(
      std::bind(&FishBotControl::_odomDataCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  // pub odom and tf
  // cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
  //     "cmd_vel",
  //     10,
  //     std::bind(&FishBotControl::_cmdVelCallback, this, std::placeholders::_1));
  // sleep(1);
  // fishbot_driver.SetFishBotSpeed(0.0, 3.1415926 / 2.0);
  // sleep(4);
  // fishbot_driver.SetFishBotSpeed(0.0, 0.0);
  // fishbot_odom_t odom;
  // fishbot_speed_t speed;
  // fishbot_driver.GetOdom(odom, speed);
}

void FishBotControl::_cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr &msg)
{
  // fishbot_driver_ send speed
}



void FishBotControl::_odomDataCallback(const fishbot_odom_t &odom,
                                       const fishbot_speed_t &speed)
{
  // odom_pub_ptr_->publish();
  // pub tf
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  // Fill in transform.transform.translation
  // Fill in transform.transform.rotation
  tf_broadcaster_->sendTransform(transform);
}

FishBotControl::~FishBotControl() {}

int main(int argc, char **argv)

{

  sleep(1);
  return 0;
}
