#ifndef UNITREE_OCR_INCLUDE_GUARD_HPP
#define UNITREE_OCR_INCLUDE_GUARD_HPP

#include <iostream>
#include <fstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

namespace unitree_ocr
{
  class TextDetector {
  private:
    cv::dnn::TextDetectionModel_EAST detector_;

    cv::dnn::TextRecognitionModel recognizer_;

  public:
    //TODO move this to CPP
    TextDetector(
      std::string detection_model_path, float confidence_threshold, float nms_threshold,
      std::string recognition_model_path
    ) 
    : detector_{cv::dnn::TextDetectionModel_EAST (detection_model_path)},
      recognizer_{cv::dnn::TextRecognitionModel (recognition_model_path)}
    {
      detector_.setConfidenceThreshold(confidence_threshold);
      detector_.setNMSThreshold(nms_threshold);
    }
  };
}

#endif