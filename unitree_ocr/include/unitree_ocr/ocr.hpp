#ifndef UNITREE_OCR_INCLUDE_GUARD_HPP
#define UNITREE_OCR_INCLUDE_GUARD_HPP


#include <string>
#include <vector>
#include <tuple>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

//TODO document

namespace unitree_ocr
{
  class TextDetector {
  private:
    cv::dnn::TextDetectionModel_EAST detector_;

    cv::dnn::TextRecognitionModel recognizer_;

    std::vector<std::vector<cv::Point>> detection_results_;

    std::vector<std::string> recognition_results_;

  public:

    TextDetector(
      std::string detection_model_path, float confidence_threshold, float nms_threshold,
      std::string recognition_model_path, std::string vocabulary_path,
      int resize_width, int resize_height
    );

    void detect(cv::Mat frame);

    std::vector<std::vector<cv::Point>> get_contours();

    std::vector<std::string> get_text();

    size_t results_size();

    std::tuple<std::vector<cv::Point>, std::string> result(size_t index);


  };

  void fourPointsTransform(const cv::Mat& frame, const cv::Point2f vertices[], cv::Mat& result);
}

#endif