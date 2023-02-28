#ifndef UNITREE_OCR_INCLUDE_GUARD_HPP
#define UNITREE_OCR_INCLUDE_GUARD_HPP


#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

//TODO document

namespace unitree_ocr
{
  class TextDetector {
  private:
    cv::dnn::TextDetectionModel_EAST detector_;

    cv::dnn::TextRecognitionModel recognizer_;

  public:

    TextDetector(
      std::string detection_model_path, float confidence_threshold, float nms_threshold,
      std::string recognition_model_path, std::string vocabulary_path,
      int resize_width, int resize_height
    ); 
  };
}

#endif