#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>

class TextDetectionSubscriber : public rclcpp::Node
{
public:
  TextDetectionSubscriber()
  : Node("text_detection_subscriber")
  {
    RCLCPP_INFO_STREAM(get_logger(), "text_detection_subscriber node started");
  }
private:
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextDetectionSubscriber>());
  rclcpp::shutdown();
  return 0;
}