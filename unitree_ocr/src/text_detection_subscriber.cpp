#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "unitree_ocr/ocr.hpp"

using unitree_ocr::TextDetector;

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

    param.description = "Confidence threshold for text detection. Must be between 0 and 1.";
    declare_parameter("detection.confidence_threshold", 0.9, param);
    auto detection_confidence_threshold =
      get_parameter("detection.confidence_threshold").get_parameter_value().get<double>();

    if (detection_confidence_threshold < 0.0 || detection_confidence_threshold > 1.0) {
      RCLCPP_ERROR_STREAM(get_logger(),
        "Invalid detection confidence threshold provided: " << detection_confidence_threshold);
      required_parameters_received = false;
    }

    param.description = "Non-maximum suppression threshold for text detection. Must be between 0 and 1.";
    declare_parameter("detection.nms_threshold", 0.4, param);
    auto detection_nms_threshold =
      get_parameter("detection.nms_threshold").get_parameter_value().get<double>();

    if (detection_nms_threshold < 0.0 || detection_nms_threshold > 1.0) {
      RCLCPP_ERROR_STREAM(get_logger(),
        "Invalid detection NMS threshold provided: " << detection_nms_threshold);
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

    detector_ = std::make_unique<TextDetector>(
      detection_model_path,
      detection_confidence_threshold,
      detection_nms_threshold,
      recognition_model_path
    );

    RCLCPP_INFO_STREAM(get_logger(), "text_detection_subscriber node started");
  }
private:
  
  std::unique_ptr<TextDetector> detector_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextDetectionSubscriber>());
  rclcpp::shutdown();
  return 0;
}