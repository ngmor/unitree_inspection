#include "unitree_ocr/ocr.hpp"

#include <iostream>
#include <fstream>
#include <vector>

namespace unitree_ocr
{
  TextDetector::TextDetector(
      std::string detection_model_path, float confidence_threshold, float nms_threshold,
      std::string recognition_model_path, std::string vocabulary_path,
      int resize_width, int resize_height
    ) 
    : detector_{cv::dnn::TextDetectionModel_EAST (detection_model_path)},
      recognizer_{cv::dnn::TextRecognitionModel (recognition_model_path)}
    {
      //ensure we have all provided file paths
      CV_Assert(detection_model_path != "");
      CV_Assert(recognition_model_path != "");
      CV_Assert(vocabulary_path != "");

      //Parameters for detection
      //TODO make inputs of parameters that aren't inputs?
      detector_.setConfidenceThreshold(confidence_threshold);
      detector_.setNMSThreshold(nms_threshold);
      double detScale = 1.0;
      cv::Size detInputSize = cv::Size(resize_width, resize_height);
      cv::Scalar detMean = cv::Scalar(123.68, 116.78, 103.94);
      bool swapRB = true;
      detector_.setInputParams(detScale, detInputSize, detMean, swapRB);

      //Load vocabulary
      std::ifstream vocab_file;
      vocab_file.open(cv::samples::findFile(static_cast<std::string>(vocabulary_path)));
      CV_Assert(vocab_file.is_open());

      //Add characters to array
      std::string vocab_line;
      std::vector<std::string> vocab;

      while (std::getline(vocab_file, vocab_line)) {
        vocab.push_back(vocab_line);
      }
      
      //Add vocab to recognizer and set decode type
      recognizer_.setVocabulary(vocab);
      recognizer_.setDecodeType("CTC-greedy");

      //Parameters for recognition
      //TODO make inputs of parameters that aren't inputs?
      double recScale = 1.0 / 127.5;
      cv::Size recInputSize = cv::Size(100,32);
      cv::Scalar recMean = cv::Scalar(127.5, 127.5, 127.5);
      recognizer_.setInputParams(recScale, recInputSize, recMean);
    }
}