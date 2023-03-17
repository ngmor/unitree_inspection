#ifndef UNITREE_OCR_INCLUDE_GUARD_HPP
#define UNITREE_OCR_INCLUDE_GUARD_HPP


#include <string>
#include <vector>
#include <tuple>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

namespace unitree_ocr
{
  /// @brief a class for detecting and recognizing text in an input image frame
  class TextDetector {
  private:
    cv::dnn::TextDetectionModel_EAST detector_;

    cv::dnn::TextRecognitionModel recognizer_;

    std::vector<std::vector<cv::Point>> detection_results_;

    std::vector<std::string> recognition_results_;

  public:

    /// @brief construct a TextDetector
    /// @param detection_model_path - Path to a binary .pb file containing trained detector network.
    /// @param confidence_threshold - Confidence threshold for considering text detected in an image region.
    /// @param nms_threshold - Non-maximum suppression threshold for detection.
    /// @param recognition_model_path - Path to a binary .onnx file containing trained CRNN text recognition model.
    /// @param vocabulary_path - Path to vocabulary benchmarks for evaluation.
    /// @param resize_width - Preprocess input image by resizing to a specific width. It should be a multiple of 32.
    /// @param resize_height - Preprocess input image by resizing to a specific height. It should be a multiple of 32.
    TextDetector(
      std::string detection_model_path, float confidence_threshold, float nms_threshold,
      std::string recognition_model_path, std::string vocabulary_path,
      int resize_width, int resize_height
    );

    /// @brief detect text within an input frame
    /// @param frame - frame to detect text in
    void detect(cv::Mat frame);

    /// @brief return the contours of the detected text from the last processed frame
    /// @return a vector containing a vector of points that represent the vertices of the contour
    std::vector<std::vector<cv::Point>> get_contours();

    /// @brief return the detected text from the last processed frame
    /// @return a vector containing the detected text
    std::vector<std::string> get_text();

    /// @brief get the number of text detections from the last processed frame
    /// @return the number of text detections
    size_t results_size();

    /// @brief get the resulting vertices and text at a certain index of the stored vectors
    /// @param index - index at which to access the vertices/text
    /// @return a tuple, where the first value is a vector with the vertices and the second
    /// value is the text detected
    std::tuple<std::vector<cv::Point>, std::string> result(size_t index);


  };

  /// @brief crop frame to get detection area
  /// @param frame - frame to crop
  /// @param vertices - vertices to crop around
  /// @param result - cropped frame
  void fourPointsTransform(const cv::Mat& frame, const cv::Point2f vertices[], cv::Mat& result);
}

#endif