import cv2
import numpy as np
from pynput import keyboard
from Pluto import pluto
import threading

class DroneController:
    def __init__(self):
        # Initialize Pluto drone object
        self.drone = pluto()

    def mapping(self, x, inMin, inMax, outMin, outMax):
        x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
        if x < outMin:
            return int(outMin)
        elif x > outMax:
            return int(outMax)
        else:
            return int(x)

    def control_loop(self):
        def on_press(key):
            try:
                if key.char == 't':
                    self.drone.rcYaw = 1000
                elif key.char == 'x':
                    self.drone.rcYaw = 2000
                elif key.char == 'z':
                    self.drone.rcPitch = 1000
                elif key.char == 'c':
                    self.drone.rcPitch = 2000
                elif key.char == 'q':
                    self.drone.rcRoll = 1000
                elif key.char == 'e':
                    self.drone.rcRoll = 2000
                elif key.char == 'a':
                    self.drone.take_off()
                    print("Takeoff")
                elif key.char == 'l':
                    self.drone.land()
                    print("Landing")
                elif key.char == 'd':
                    self.drone.disarm()
                elif key.char == 's':
                    self.drone.decrease_height()
                elif key.char == 'w':
                    self.drone.increase_height()    
            except AttributeError:
                pass

        def on_release(key):
            try:
                if key.char == 'a' or key.char == 'd':
                    self.drone.rcYaw = 1500
                elif key.char == 'w' or key.char == 's':
                    self.drone.rcPitch = 1500
                elif key.char == 'q' or key.char == 'e':
                    self.drone.rcRoll = 1500
            except AttributeError:
                pass

        # Collect events until released
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

# Create an instance of the DroneController class
drone_controller = DroneController()

# Start the control loop in a separate thread
control_thread = threading.Thread(target=drone_controller.control_loop)
control_thread.daemon = True
control_thread.start()


# Global variables to store cursor position
x_pos, y_pos = 0, 0
def draw_grid(frame):
    # Get frame dimensions
    height, width, _ = frame.shape

    # Calculate quadrant dimensions
    quadrant_height = height // 3
    quadrant_width = width // 3

    # Draw vertical lines to split into 3 columns
    cv2.line(frame, (quadrant_width, 0), (quadrant_width, height), (255, 0, 0), 1)
    cv2.line(frame, (2 * quadrant_width, 0), (2 * quadrant_width, height), (255, 0, 0), 1)

    # Draw horizontal lines to split into 3 rows
    cv2.line(frame, (0, quadrant_height), (width, quadrant_height), (255, 0, 0), 1)
    cv2.line(frame, (0, 2 * quadrant_height), (width, 2 * quadrant_height), (255, 0, 0), 1)

def track_yellow(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range of yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # If contours are found
    if contours:
        # Find the largest contour
        max_contour = max(contours, key=cv2.contourArea)
        
        # Get the centroid of the largest contour
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Display centroid position
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            
            # Control the drone based on centroid position within quadrants
            if cx < 210:  # Move left
                drone_controller.drone.rcRoll = 1400
                print("Moving left:", drone_controller.drone.rcRoll)
            elif cx > 400:  # Move right
                drone_controller.drone.rcRoll = 1600
                print("Moving right:", drone_controller.drone.rcRoll)
            else:  # Center position in X direction
                drone_controller.drone.rcRoll = 1500

            if cy < 160:  # Move up
                drone_controller.drone.rcPitch = 1600
                print("Moving forward:", drone_controller.drone.rcPitch)
            elif cy > 320:  # Move down
                drone_controller.drone.rcPitch = 1400
                print("Moving backward:", drone_controller.drone.rcPitch)
            else:  # Center position in Y direction
                drone_controller.drone.rcPitch = 1500
    
    # If no contours are found, stop the drone's movements
    else:
        drone_controller.drone.rcRoll = 1500
        drone_controller.drone.rcPitch = 1500

def mapping(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Draw gridlines on the frame
    draw_grid(frame)
    
    # Track yellow object
    track_yellow(frame)

    # Display the frame
    cv2.imshow('Webcam with Yellow Tracking and Grid', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release webcam and close all windows
cap.release()
cv2.destroyAllWindows()
