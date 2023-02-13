#!/usr/bin/env python3
# https://nanonets.com/blog/deep-learning-ocr/
import numpy as np
import cv2
from imutils.object_detection import non_max_suppression
import pytesseract
from matplotlib import pyplot as plt

image_file_path = 'images/oled.jpg'
model_file_path = 'models/frozen_east_text_detection.pb'
min_confidence = 0.5
width_processed = 320 #pixels
height_processed = 320 #pixels
im2str_configuration = ("-l eng --oem 1 --psm 7") # tesseract configuration to recognize text from interest region

# load pre-trained EAST model
net = cv2.dnn.readNet(model_file_path)

layer_names = [
    "feature_fusion/Conv_7/Sigmoid", # layer for probability scores for whether a region contains text
    "feature_fusion/concat_3", # layer for coordinates of bounding box
]

camera = cv2.VideoCapture(0)

while (True):

    # read image
    # image_orig = cv2.imread(image_file_path)
    ret, image_orig = camera.read()
    image_processed = image_orig.copy()

    # get original dimensions
    (height_orig, width_orig) = image_orig.shape[:2]

    # calculate ratio between original dimensions and processing dimensions
    width_ratio = width_orig / float(width_processed)
    height_ratio = height_orig / float(height_processed)

    # resize image to processing dimensions
    image_processed = cv2.resize(image_processed, (width_processed, height_processed))

    # construct a blob from the image to forward pass it to EAST model
    blob = cv2.dnn.blobFromImage(image_processed, 1.0, (width_processed, height_processed),
                                (123.68, 116.78, 103.94), swapRB=True, crop=False)


    # forward pass the blob to get the desired output layers
    net.setInput(blob)
    (scores, geometry) = net.forward(layer_names)

    # Return bounding boxes and probability score if it is more than minimum confidence
    #TODO redo using numpy arrays?
    #TODO implement rotated bounding boxes? this only does horizontal bounding boxes
    def predictions(prob_score, geo):
        (num_rows, num_cols) = prob_score.shape[2:4]
        boxes = []
        confidence_vals = []

        # loop over rows of probability scores
        for i in range(0, num_rows):
            scores_data = prob_score[0,0,i]
            x0 = geo[0,0,i]
            x1 = geo[0,1,i]
            x2 = geo[0,2,i]
            x3 = geo[0,3,i]
            angles = geo[0,4,i]
        
            # loop over columns of probability scores
            for j in range(0, num_cols):
                
                # ignore data with a score less than the minimum confidence
                if scores_data[j] < min_confidence:
                    continue


                (x_offset, y_offset) = (j*4.0, i*4.0)

                # extract rotation angle for prediction and calculate sine/cosine
                angle = angles[j]
                cos = np.cos(angle)
                sin = np.sin(angle)

                #get dimensions of bounding box with geometry volume
                height = x0[j] + x2[j]
                width = x1[j] + x3[j]

                # compute start and end for text prediction bounding box
                x_end = int(x_offset + cos*x1[j] + sin*x2[j])
                y_end = int(y_offset - sin*x1[j] + cos*x2[j])
                x_start = int(x_end - width)
                y_start = int(y_end - height)

                boxes.append((x_start, y_start, x_end, y_end))
                confidence_vals.append(scores_data[j])

        # return bounding boxes and confidence values
        return (boxes, confidence_vals)

    # find predictions and apply non-maxima suppression
    (boxes, confidence_vals) = predictions(scores, geometry)

    boxes = non_max_suppression(np.array(boxes), probs=confidence_vals)

    # Text detection and recognition
    image_result = image_orig.copy()

    # loop over bounding boxes
    for (x_start, y_start, x_end, y_end) in boxes:

        # TODO turn scaling into a class
        # scale coordinates back to original image size
        x_start = int(x_start*width_ratio)
        y_start = int(y_start*height_ratio)
        x_end = int(x_end*width_ratio)
        y_end = int(y_end*height_ratio)

        # extract region of interest
        region = image_orig[y_start:y_end, x_start:x_end]

        # extract text from image
        text = pytesseract.image_to_string(region, config=im2str_configuration)

        # Displaying text
        text = "".join([x if ord(x) < 128 else "" for x in text]).strip()
        
        # Add to image
        cv2.rectangle(image_result, (x_start, y_start), (x_end, y_end),
            (0, 0, 255), 2)
        cv2.putText(image_result, text, (x_start, y_start - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0, 255), 2)

    # show images
    cv2.imshow('Result', image_result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera.release()
cv2.destroyAllWindows()