#include <pigpiod_if2.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace modot_raspi
{
class TactileNotifier : public rclcpp::Node
{
public:
  explicit TactileNotifier(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  TactileNotifier(const std::string& name, const std::string& ns,
                  const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

private:
  void enableObstacleNotificationCallback(const rclcpp::Parameter& p);

  void obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool enable_obstacle_notification_;

  int wing_motor_relay_pin_;

  int pi_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> parameter_callback_handles_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
};
}  // namespace modot_raspi
