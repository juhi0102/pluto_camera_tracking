import cv2
import numpy as np
import pylwdrone
from pylwdrone import LWDrone
import ffmpeg
import sys
import threading
from pynput import keyboard

from Pluto import pluto

from pid import PID

from Joystick_controls import XboxController
from PlutoMultiwii import *

class DroneController:
    def __init__(self):
        # Initialize Xbox controller and Pluto drone objects
        self.joy = XboxController()
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
        while True:
            [x, y, a, b, A, B, X, Y, rb, lb, rt, lt, ld, rd, ud, dd] = self.joy.read()

            self.drone.rcThrottle = self.mapping(y, 1, -1, 1000, 2000)
            self.drone.rcYaw = self.mapping(x, -1, 1, 1000, 2000)
            self.drone.rcPitch = self.mapping(b, 1, -1, 1000, 2000)
            self.drone.rcRoll = self.mapping(a, -1, 1, 1000, 2000)

            if A:
                self.drone.arm()
                print("Arming")
            elif B:
                self.drone.disarm()
                print("Disarming")
            elif Y:
                self.drone.take_off()
                print("Takeoff")
            elif X:
                self.drone.land()
                print("Landing")



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
                # Find the largest contour
                max_contour = max(contours, key=cv2.contourArea)
                
                # Get the centroid of the largest contour
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Display centroid position
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    offset_x = cx - frame.shape[1] // 2
                    # offset_y = cy - frame.shape[0] // 2

                    Kp_roll = 1.0
                    Ki_roll = 0.01
                    Kd_roll = 0.01

                    # Kp_pitch = 0.1
                    # Ki_pitch = 0.01
                    # Kd_pitch = 0.01

                    # Define setpoints (desired positions)
                    roll_setpoint = 0
                    pitch_setpoint = 0

                    # Define time step (dt)
                    dt = 0.1
                    pid_roll = PID()
                    # pid_pitch = PID()

                    # Set PID parameters
                    pid_roll.set_parameters(Kp_roll, Ki_roll, Kd_roll, roll_setpoint, dt)
                    # pid_pitch.set_parameters(Kp_pitch, Ki_pitch, Kd_pitch, pitch_setpoint, dt)

                    # Within your loop
                    pid_roll_output = pid_roll.update(offset_x)
                    # pid_pitch_output = pid_pitch.update(offset_y)

                    # Apply the outputs to drone control
                    if abs(offset_x) > 10:
                        drone_controller.drone.rcRoll = 1500 - int(pid_roll_output)  # Negate the output
                    else:
                        drone_controller.drone.rcRoll = 1500

                    # if abs(offset_y) > 10:
                    #     me.rcPitch = 1500 - int(pid_pitch_output)  # Negate the output
                    # else:
                    #     me.rcPitch = 1500



                else:
                    drone_controller.drone.rcRoll = 1500
                    # me.rcPitch = 1500

# Create an instance of the DroneController class
drone_controller = DroneController()
# Start the joystick control loop in a separate thread
joystick_thread = threading.Thread(target=drone_controller.control_loop)
joystick_thread.daemon = True
joystick_thread.start()




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
