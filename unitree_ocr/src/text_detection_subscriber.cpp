#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "unitree_ocr/ocr.hpp"

class TextDetectionSubscriber : public rclcpp::Node
{
public:
  TextDetectionSubscriber()
  : Node("text_detection_subscriber")
  {
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The file path to the text detection model (REQUIRED).";
    declare_parameter("detection.model_path", "", param);
    auto detection_model_path =
      get_parameter("detection.model_path").get_parameter_value().get<std::string>();

    if (detection_model_path == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No detection model path provided.");
      required_parameters_received = false;
    }

    param.description = "The file path to the text recognition model (REQUIRED).";
    declare_parameter("recognition.model_path", "", param);
    auto recognition_model_path =
      get_parameter("recognition.model_path").get_parameter_value().get<std::string>();

    if (recognition_model_path == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No recognition model path provided.");
      required_parameters_received = false;
    }

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

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