"""
  Thesis proposed by Group : Prex B. Luciano, Daniela Alonzo, Jimbo Carlo Delfin, Jedric C. Cascante
  
  RaWaste Sorting Robotic Arm - ( Description of the Project )
  
"""
import argparse
import sys
import time
import utils
import cv2
import math
import serial
import warnings
from PIL import Image
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

#Fetch error the object cannot be divided by 0 and ignore it
warnings.filterwarnings("ignore", category=RuntimeWarning)

#
port = '/dev/ttyUSB0'  
rate = 9600

ser = serial.Serial(port,rate)
ser.flushInput()
comp_list = ["Done Moving\r\n", "Connected to Arduino\r\n"]


def run(model: str, camera_id: int) -> None:
  
  width = 640
  height = 480


  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (255, 255, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 23

  # Initialize the object detection model
  base_options = core.BaseOptions(file_name=model)
  detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.5)
  options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)
  
  done = 1
  stop = 0
  objectAngle =0
  cardboardCount = 0
  glassCount = 0
  metalCount = 0
  paperCount = 0
  plasticCount = 0

  # Continuously capture images from the camera and run inference
  while True:
  #while cap.isOpened():
    
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Camera not Connected'
      )

    counter += 1
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)
    
    # Run object detection estimation using the model.
    detection_result = detector.detect_with_metadata(input_tensor,metadata)
    
    for detection in detection_result.detections:
       # Get the Results
       bbox = detection.bounding_box
       objectClass = detection.categories[0].category_name
       
       # Calculate the center point of the bounding box
       origin_x = bbox.origin_x
       origin_y = bbox.origin_y
       center_x = origin_x + (bbox.width / 2)
       center_y = origin_y + (bbox.height / 2)
       # DISTANCE
       objDistance = int(math.sqrt(((center_x - 480)**2)+((center_y - 845)**2)))
                                                                    #320
       if objDistance <= 600:
         inputDistance = ' 1'
       if objDistance >= 601 and objDistance <= 650:
         inputDistance = ' 2'
       if objDistance >= 651 and objDistance <= 1000:
         inputDistance = ' 3'
       if objDistance >= 1001:
         inputDistance = ' 4'
       # ANGLE (X)
       objectAngle = int(math.atan2(center_y - 845, center_x -320)*180/math.pi)
       # Convert the Angle into 0 - 180 format
       
       if objectAngle > 0:
         objectAngle = abs(objectAngle -180)
       if objectAngle == 90:
          objectAngle = 0
       if objectAngle < 0:
          objectAngle = -objectAngle
# Print the center point of the bounding box
       inputAngle = ' ' + str(objectAngle)
       
       if done == 1:
         if objectClass == "paper":
           paperCount += 1
         if objectClass == "plastic":
           plasticCount += 1
           print("Counted :", objectClass, "for #", plasticCount)
           print("Object Distance:", objDistance)
         if objectClass == "metal":
           metalCount += 1
         if objectClass == "glass":
           glassCount += 1
         if objectClass == "cardboard":
           cardboardCount += 1

    if ser.inWaiting()>0:
      
      #Get the Input 
      inputValue = ser.readline()
      print(inputValue.decode())
      
      # If the arduino is done moving from arduino 
      if inputValue.decode() == "Done Moving\r\n":   
        done = 1 
      
      if inputValue.decode() in comp_list:
        if cardboardCount >= 20 and done == 1 and objectAngle != 0:
          print("Counted :", objectClass, "for #", cardboardCount)
          print("Object detected at:", center_x, center_y)
          print("Angle from the arm:",objectAngle)
          
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputDistance, 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputAngle, 'utf-8'))
          
          stop = 1
          done = 0

        """if glassCount >= 20 and done == 1 and objectAngle != 0:
          print("Counted :", objectClass, "for #", glassCount)
          print("Object detected at:", center_x, center_y)
          print("Angle from the arm:",inputAngle)
          
          ser.write(bytes(str(2), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputDistance, 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputAngle, 'utf-8'))
          
          stop = 1
          done = 0
          time.sleep(60)
        """
        if paperCount >= 30 and done == 1 and objectAngle != 0:
          print("Counted :", objectClass, "for #", paperCount)
          print("Object detected at:", center_x, center_y)
          print("Angle from the arm:",inputAngle)
          
          ser.write(bytes(str(3), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputDistance, 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputAngle, 'utf-8'))
          
          stop = 1
          done = 0
          time.sleep(20)

        if metalCount >= 30 and done == 1 and objectAngle != 0:
          print("Counted :", objectClass, "for #", metalCount)
          print("Object detected at:", center_x, center_y)
          print("Angle from the arm:",inputAngle)
          
          ser.write(bytes(str(4), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputDistance, 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputAngle, 'utf-8'))
          
          
          stop = 1
          done = 0
          time.sleep(20)
        if plasticCount >= 30 and done == 1 and objectAngle != 0:
          print("Counted :", objectClass, "for #", plasticCount)
          print("Object detected at:", center_x, center_y)
          print("Angle from the arm:",inputAngle)
          ser.write(bytes(str(5), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(str(1), 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputDistance, 'utf-8'))
          ser.write(bytes(' ', 'utf-8'))
          ser.write(bytes(inputAngle, 'utf-8'))

          stop = 1
          done = 0
          time.sleep(20)
      if stop == 1:
        cardboardCount = 0
        glassCount = 0
        metalCount = 0
        paperCount = 0
        plasticCount = 0
        stop = 0
    # Draw keypoints and edges on input image
    image = utils.visualize(image, detection_result,objectAngle)
    
    #print(done)
    #print(plasticCount, metalCount, glassCount, paperCount, cardboardCount)
    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
      break
    cv2.imshow('RAWASTE_DETECTOR', image)
  cap.release()
  cv2.destroyAllWindows()


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Name & Path of the Pre-Trained Detection Model',
      required=False,
      default='dogs.tflite')
      
  parser.add_argument(
      '--cameraId', 
      help='Camera Id if using another type of camera', 
      required=False, 
      type=int, 
      default=0)
      
  args = parser.parse_args()

  run(args.model, int(args.cameraId))

if __name__ == '__main__':
  main()
