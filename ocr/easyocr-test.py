#!/usr/bin/env python3
# https://pyimagesearch.com/2020/09/14/getting-started-with-easyocr-for-optical-character-recognition/
from easyocr import Reader
import cv2

def cleanup_text(text):
    # strip out non-ASCII text so we can draw the text on the image
    # using OpenCV
    return "".join([c if ord(c) < 128 else "" for c in text]).strip()

langs = ["en"]

camera = cv2.VideoCapture(0)

reader = Reader(langs, gpu=False)

while (True):
    # image_orig = cv2.imread(image_file_path)
    ret, image_orig = camera.read()

    results = reader.readtext(image_orig)

    image_result = image_orig.copy()

    #loop over results
    for (bbox, text, prob) in results:
        # get coordinates from bounding box
        (tl, tr, br, bl) = bbox
        tl = (int(tl[0]), int(tl[1]))
        tr = (int(tr[0]), int(tr[1]))
        br = (int(br[0]), int(br[1]))
        bl = (int(bl[0]), int(bl[1]))

        text = cleanup_text(text)

        image_result = cv2.rectangle(image_result, tl, br, (0,255,0), 2)
        image_result = cv2.putText(image_result, text, (tl[0], tl[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # show images
    cv2.imshow('Result', image_result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera.release()
cv2.destroyAllWindows()