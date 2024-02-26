#include "modot_raspi/tactile_notifier.hpp"

#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::placeholders;

namespace modot_raspi
{
TactileNotifier::TactileNotifier(const rclcpp::NodeOptions& node_options)
  : TactileNotifier("tactile_notifier", "", node_options)
{
}

TactileNotifier::TactileNotifier(const std::string& name, const std::string& ns,
                                 const rclcpp::NodeOptions& node_options)
  : Node(name, ns, node_options), enable_obstacle_notification_(false), pi_(-1)
{
  pi_ = pigpio_start(nullptr, nullptr);

  if (pi_ < 0)
  {
    throw std::runtime_error("Failed to connect to the pigpio daemon");
  }

  set_mode(pi_, WING_MOTOR_RELAY_PIN, PI_OUTPUT);

  declare_parameter("enable_obstacle_notification", false);
  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
      "enable_obstacle_notification", std::bind(&TactileNotifier::enableObstacleNotificationCallback, this, _1)));

  obstacle_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "obstacle_detected", 10, std::bind(&TactileNotifier::obstacleDetectedCallback, this, _1));
}

void TactileNotifier::enableObstacleNotificationCallback(const rclcpp::Parameter& p)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
  {
    RCLCPP_ERROR(get_logger(), "Invalid parameter type");
    return;
  }

  enable_obstacle_notification_ = p.as_bool();

  gpio_write(pi_, WING_MOTOR_RELAY_PIN, PI_LOW);
}

void TactileNotifier::obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (enable_obstacle_notification_)
  {
    gpio_write(pi_, WING_MOTOR_RELAY_PIN, msg->data ? PI_HIGH : PI_LOW);
  }
}
}  // namespace modot_raspi

RCLCPP_COMPONENTS_REGISTER_NODE(modot_raspi::TactileNotifier)
