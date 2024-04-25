import cv2
import numpy as np
import pylwdrone
from pynput import keyboard, mouse
from pylwdrone import LWDrone
import ffmpeg
import sys
import threading

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

    def mouse_event(self, x, y, dx=None, dy=None):
        # Additional movement based on cursor position
        if x < 200:  # Move left
            self.drone.rcRoll = self.mapping(x, 0, 200, 1300, 1500)
            print("Moving left:", self.drone.rcRoll)
        elif x > 500:  # Move right
            self.drone.rcRoll = self.mapping(x, 500, 700, 1500, 1700)
            print("Moving right:", self.drone.rcRoll)
        else:  # Center position
            self.drone.rcRoll = 1500
            print("Center position")

        if y < 200:  # Move up
            self.drone.rcPitch = self.mapping(y, 150, 0, 1500, 1700)
            print("Moving forward:", self.drone.rcPitch)
        elif y > 300:  # Move down
            self.drone.rcPitch = self.mapping(y, 300, 500, 1500, 1300)
            print("Moving backward:", self.drone.rcPitch)

# Initialize drone controller
drone_controller = DroneController()

# Start control loop in a separate thread
control_thread = threading.Thread(target=drone_controller.control_loop)
control_thread.daemon = True
control_thread.start()

# Set up mouse event listener
mouse_listener = mouse.Listener(on_move=drone_controller.mouse_event)
mouse_listener.start()

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


        # Display video stream with grid
        cv2.imshow(window_name, frame)

        # Check for 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except ffmpeg.Error as e:
        print('An error occurred:', e.stderr.decode(), file=sys.stderr)
cv2.destroyAllWindows()
drone1.stop_video_stream()