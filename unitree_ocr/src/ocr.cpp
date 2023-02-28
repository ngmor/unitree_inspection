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

    void TextDetector::detect(cv::Mat frame) {
      //clear previous results
      detection_results_.clear();

      //Detect text in frame
      detector_.detect(frame, detection_results_);
      
      //Initialize recognition results
      recognition_results_ = std::vector<std::string>(detection_results_.size(), "");

      //Recognize text in detected regions
      for (size_t i = 0; i < detection_results_.size(); i++) {
        const auto & quadrangle = detection_results_[i];

        CV_CheckEQ(quadrangle.size(), (size_t)4, "Detection quadrangle size check");

        std::vector<cv::Point2f> quadrangle_2f;

        for (size_t j = 0; j < 4; j++) {
          quadrangle_2f.emplace_back(quadrangle[j]);
        }

        //Crop frame to get detection area
        cv::Mat cropped;
        fourPointsTransform(frame, &quadrangle_2f[0], cropped);

        //Store recognized text in results array
        recognition_results_[i] = recognizer_.recognize(cropped);
      }
    }

    std::vector<std::vector<cv::Point>> TextDetector::get_contours() {return detection_results_;}

    std::vector<std::string> TextDetector::get_text() {return recognition_results_;}

    size_t TextDetector::results_size() {return detection_results_.size();}

    std::tuple<std::vector<cv::Point>, std::string> TextDetector::result(size_t index) {
      return {
        detection_results_.at(index),
        recognition_results_.at(index)
      };
    }


    void fourPointsTransform(const cv::Mat& frame, const cv::Point2f vertices[], cv::Mat& result) {
      const cv::Size outputSize = cv::Size(100, 32);

      cv::Point2f targetVertices[4] = {
          cv::Point(0, outputSize.height - 1),
          cv::Point(0, 0),
          cv::Point(outputSize.width - 1, 0),
          cv::Point(outputSize.width - 1, outputSize.height - 1)
      };
      cv::Mat rotationMatrix = cv::getPerspectiveTransform(vertices, targetVertices);

      cv::warpPerspective(frame, result, rotationMatrix, outputSize);
    }
}