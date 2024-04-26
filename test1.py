import cv2
import numpy as np
import pylwdrone
from pylwdrone import LWDrone
import ffmpeg
import sys
import threading
from pynput import keyboard

from Pluto import pluto

class DroneController:
    def __init__(self):
        # Initialize Pluto drone object
        self.drone = pluto()

    def mapping(self, value, in_min, in_max, out_min, out_max):
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def control_loop(self):
        def on_press(key):
            try:
                if key.char == 'a':
                    self.drone.rcYaw = 1000
                elif key.char == 'd':
                    self.drone.rcYaw = 2000
                elif key.char == 'z':
                    self.drone.rcPitch = 1000
                elif key.char == 'c':
                    self.drone.rcPitch = 2000
                elif key.char == 'q':
                    self.drone.rcRoll = 1000
                elif key.char == 'e':
                    self.drone.rcRoll = 2000
                elif key.char == 't':
                    self.drone.take_off()
                    print("Takeoff")
                elif key.char == 'l':
                    self.drone.land()
                    print("Landing")
                elif key.char == 'x':
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
        with keyboard.Listener(on_press=on_press, on_release=on_release) as key_listener:
            key_listener.join()

    def track_blue(self, frame):
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define range of blue color in HSV
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours are found
        if contours:
            # Find the largest contourq
            max_contour = max(contours, key=cv2.contourArea)
            
            # Get the centroid of the largest contour
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Display centroid position
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Calculate the offset from the center
                offset_x = cx - frame.shape[1] // 2
                offset_y = cy - frame.shape[0] // 2
                
                # Control the drone based on centroid position
                if abs(offset_x) > 10:  # Adjust drone's roll
                    self.drone.rcRoll = 1500 + int(offset_x * 0.2)
                    print("Adjusting roll:", self.drone.rcRoll)
                else:  # Centered in X direction
                    self.drone.rcRoll = 1500
                
                if abs(offset_y) > 10:  # Adjust drone's pitch
                    self.drone.rcThrottle = 1500 + int(offset_y * 0.2)
                    print("Adjusting pitch:", self.drone.rcThrottle)
                else:  # Centered in Y direction
                    self.drone.rcThrottle = 1500
        
        # If no contours are found, stop the drone's movements
        else:
            self.drone.rcRoll = 1500
            self.drone.rcThrottle = 1500

# Initialize drone controller
drone_controller = DroneController()

# Start control loop in a separate thread
control_thread = threading.Thread(target=drone_controller.control_loop)
control_thread.daemon = True
control_thread.start()

# Initialize drone video stream
drone1 = pylwdrone.LWDrone()
drone1.set_time()
window_name = 'Drone Video Stream'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

for packet in drone1.start_video_stream():
    try:
        out, _ = (
            ffmpeg.input('pipe:0')
            .output('pipe:', format='rawvideo', pix_fmt='bgr24')
            .run(input=packet.frame_bytes, capture_stdout=True, capture_stderr=True)
        )
        frame = np.frombuffer(out, np.uint8)
        height, width = 1152, 2048
        frame = frame.reshape((height, width, 3)).copy()  # Make a writable copy of the frame

        # Draw grid lines
        grid_color = (0, 255, 0)  # Green color for grid lines
        grid_thickness = 1  # Thickness of grid lines
        # Vertical grid lines
        cv2.line(frame, (width // 3, 0), (width // 3, height), grid_color, grid_thickness)
        cv2.line(frame, (2 * width // 3, 0), (2 * width // 3, height), grid_color, grid_thickness)
        # Horizontal grid lines
        cv2.line(frame, (0, height // 3), (width, height // 3), grid_color, grid_thickness)
        cv2.line(frame, (0, 2 * height // 3), (width, 2 * height // 3), grid_color, grid_thickness)

        # Track blue color and control the drone
        drone_controller.track_blue(frame)

        # Check for 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

       

        # Display video stream with grid and detected object circle
        cv2.imshow(window_name, frame)

    except ffmpeg.Error as e:
        print('An error occurred:', e.stderr.decode(), file=sys.stderr)

cv2.destroyAllWindows()
drone1.stop_video_stream()
