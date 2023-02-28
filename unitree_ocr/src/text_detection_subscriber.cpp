#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"
#include "unitree_ocr/ocr.hpp"

using unitree_ocr::TextDetector;

const std::string WINDOW_NAME = "Optical Character Recognition";

class TextDetectionSubscriber : public rclcpp::Node
{
public:
  TextDetectionSubscriber()
  : Node("text_detection_subscriber")
  {
    //Parameters
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

    param.description = "The file path to the text recognition vocabulary (REQUIRED).";
    declare_parameter("recognition.vocabulary_path", "", param);
    auto recognition_vocabulary_path =
      get_parameter("recognition.vocabulary_path").get_parameter_value().get<std::string>();

    if (recognition_model_path == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No recognition model path provided.");
      required_parameters_received = false;
    }

    param.description = "Preprocess input image by resizing to a specific width. Should be a multiple of 32.";
    declare_parameter("recognition.resize_width", 320, param);
    auto recognition_resize_width =
      get_parameter("recognition.resize_width").get_parameter_value().get<int>();

    param.description = "Preprocess input image by resizing to a specific height. Should be a multiple of 32.";
    declare_parameter("recognition.resize_height", 320, param);
    auto recognition_resize_height =
      get_parameter("recognition.resize_height").get_parameter_value().get<int>();

    param.description = "Swap red and blue channels in input image.";
    declare_parameter("swap_rb", true, param);
    swap_rb_ = get_parameter("swap_rb").get_parameter_value().get<bool>();

    param.description = "Size multiplier for displayed image.";
    declare_parameter("display_size_multiplier", 1.0, param);
    display_size_multiplier_ = get_parameter("display_size_multiplier").get_parameter_value().get<double>();

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Subscribers
    sub_image_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image",
          std::bind(&TextDetectionSubscriber::image_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

    detector_ = std::make_unique<TextDetector>(
      detection_model_path,
      detection_confidence_threshold,
      detection_nms_threshold,
      recognition_model_path,
      recognition_vocabulary_path,
      recognition_resize_width,
      recognition_resize_height
    );

    cv::namedWindow(WINDOW_NAME);

    RCLCPP_INFO_STREAM(get_logger(), "text_detection_subscriber node started");
  }

  ~TextDetectionSubscriber()
  {
    cv::destroyAllWindows();
  }

private:
  std::shared_ptr<image_transport::CameraSubscriber> sub_image_;

  std::unique_ptr<TextDetector> detector_;
  bool swap_rb_;
  double display_size_multiplier_;
  

  void image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    cv::Mat frame = cv_bridge::toCvCopy(img, img->encoding)->image;

    //swap red and blue channels if enabled
    if (swap_rb_) {
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    }

    //perform text detection
    detector_->detect(frame);

    //draw text on image
    for (size_t i = 0; i < detector_->results_size(); i++) {
      auto [quadrangle, text] = detector_->result(i);
      cv::putText(frame, text, quadrangle[3], cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
    }

    //Draw contours on image
    cv::polylines(frame, detector_->get_contours(), true, cv::Scalar(0, 255, 0), 2);

    //Resize for output
    cv::resize(frame, frame, cv::Size(
      static_cast<size_t>(frame.size().width*display_size_multiplier_),
      static_cast<size_t>(frame.size().height*display_size_multiplier_)
    ), cv::INTER_LINEAR);

    //Show image
    cv::imshow(WINDOW_NAME, frame);
    cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextDetectionSubscriber>());
  rclcpp::shutdown();
  return 0;
}