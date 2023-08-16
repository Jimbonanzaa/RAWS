"""
  Thesis proposed by Group : Prex B. Luciano, Daniela Alonzo, Jimbo Carlo Delfin, Jedric C. Cascante
  
  RaWaste Sorting Robotic Arm - ( Description of the Project )
  
  
"""

import cv2
import numpy as np
from tflite_support.task import processor

_MARGIN = 10  #px
_ROW_SIZE = 10 #px
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)


def visualize(
    image: np.ndarray,
    detection_result: processor.DetectionResult,
    objectAngle: int
) -> np.ndarray:

  for detection in detection_result.detections:
    
    #create circle where the arm is
    cv2.circle(image, (320, 480), 5, (0, 0, 255), -1)
    
    #Draw circles
    cv2.circle(image, (320, 480), 475, (0, 0, 255), 1, 8, 0)
    cv2.circle(image, (320, 480), 375, (0, 0, 255), 1, 8, 0)
    cv2.circle(image, (320, 480), 275, (0, 0, 255), 1, 8, 0)	
    cv2.circle(image, (320, 480), 175, (0, 0, 255), 1, 8, 0)
    
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)
    
    #Draw Center Point 
    origin_x = bbox.origin_x
    origin_y = bbox.origin_y
    center_x = int(origin_x + (bbox.width / 2))
    center_y = int(origin_y + (bbox.height / 2))
    cv2.circle(image, (center_x, center_y), 3, _TEXT_COLOR, -1)
    
    cv2.line(image, (center_x, center_y), (320, 480), (0, 0, 255), 1)
    cv2.putText(image, str(objectAngle), (310, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    
    # Draw label and score
    category = detection.categories[0]
    category_name = category.category_name
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'
    text_location = (_MARGIN + bbox.origin_x,
                     _MARGIN + _ROW_SIZE + bbox.origin_y)
    cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)

  return image
