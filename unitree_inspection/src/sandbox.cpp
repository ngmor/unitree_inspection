#include "rclcpp/rclcpp.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"

class Sandbox : public rclcpp::Node
{
public:
  Sandbox()
  : Node("sandbox")
  {

    RCLCPP_INFO_STREAM(get_logger(), "sandbox node started");
  }

private:

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sandbox>());
  rclcpp::shutdown();
  return 0;
}