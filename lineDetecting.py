import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import serial

# GPIO 설정
led_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

def process_frame(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Apply morphological operations to remove small noises and connect lines
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    
    # Use the mask to get the yellow regions
    yellow_only = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Convert the result to grayscale
    gray = cv2.cvtColor(yellow_only, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Use Canny edge detection
    edges = cv2.Canny(blur, 50, 150)
    
    # Define a region of interest (ROI) to focus on the lanes
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, 0),
        (0, 0),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # Use Hough transform to detect lines
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, maxLineGap=50)
    
    if lines is not None:
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
            
            if slope < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)
        
        def average_lines(lines):
            if len(lines) == 0:
                return None
            x_coords = []
            y_coords = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_coords.extend([x1, x2])
                y_coords.extend([y1, y2])
            poly = np.polyfit(x_coords, y_coords, 1)
            x_start = int(np.min(x_coords))
            x_end = int(np.max(x_coords))
            y_start = int(poly[0] * x_start + poly[1])
            y_end = int(poly[0] * x_end + poly[1])
            return [[x_start, y_start, x_end, y_end]]
        
        left_line = average_lines(left_lines)
        right_line = average_lines(right_lines)
        
   
        
        if left_line is not None and right_line is not None:
            for x1, y1, x2, y2 in left_line:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
            for x1, y1, x2, y2 in right_line:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
            
            x1_left, y1_left, x2_left, y2_left = left_line[0]
            x1_right, y1_right, x2_right, y2_right = right_line[0]
            
            left_bottom = (x1_left + x2_left) 
            right_bottom = (x1_right + x2_right)
            bottom_center = (left_bottom + right_bottom) // 4
            
            left_top = (x1_left + x2_left) 
            right_top = (x1_right + x2_right) 
            top_center = (left_top + right_top) //4
            
            cv2.line(frame, (bottom_center, height), (top_center, 0), (0, 255, 0), 3)
            return bottom_center, top_center, width // 2
    return None, None, None

# Use the camera
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Get the frames per second (FPS) of the camera
fps = cap.get(cv2.CAP_PROP_FPS)
delay = int(1000 / fps)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break
    
    bottom_center, top_center, frame_center = process_frame(frame)
    
    # Display the processed frame
    cv2.imshow('Processed Frame', frame)
    
    # Turn on LED if lane detected
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        GPIO.output(led_pin, GPIO.LOW)
        break
    else:
        GPIO.output(led_pin, GPIO.HIGH)
        
    if bottom_center is not None and top_center is not None:
        if bottom_center < frame_center - 10:
            ser.write(b'2')
            print("left")
        elif bottom_center > frame_center + 10:
            ser.write(b'3')
            print("right")
        else:
            ser.write(b'1')
            print("foward")
    else:
        ser.write(b'1')
        print("foward")
        
    

cap.release()
cv2.destroyAllWindows()

