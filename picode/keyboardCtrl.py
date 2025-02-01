from picarx import Picarx
from time import sleep
from evdev import InputDevice, categorize, ecodes

# Global variables for pan and tilt angles
pan_angle = 0
tilt_angle = 0
velocity = 0
turn = 0

def handle_input(device):
    global velocity, turn, pan_angle, tilt_angle
    for event in device.read():
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keystate == key_event.key_down:  # When key is pressed
                if key_event.keycode == 'KEY_W':  # Move forward
                    velocity = 10
                elif key_event.keycode == 'KEY_S':  # Move backward
                    velocity = -10
                elif key_event.keycode == 'KEY_A':  # Turn left
                    turn = -30
                elif key_event.keycode == 'KEY_D':  # Turn right
                    turn = 30
                elif key_event.keycode == 'KEY_I':  # Head up
                    tilt_angle += 10
                elif key_event.keycode == 'KEY_K':  # Head down
                    tilt_angle -= 10
                elif key_event.keycode == 'KEY_J':  # Turn head left
                    pan_angle -= 10
                elif key_event.keycode == 'KEY_L':  # Turn head right
                    pan_angle += 10
                elif key_event.keycode == 'KEY_ESC':  # Exit program on Escape
                    return False
            elif key_event.keystate == key_event.key_up:  # When key is released
                if key_event.keycode == 'KEY_W' or key_event.keycode == 'KEY_S':
                    velocity = 0
                if key_event.keycode == 'KEY_A' or key_event.keycode == 'KEY_D':
                    turn = 0
    return True

if __name__ == "__main__":
    try:
        px = Picarx()
        
        # Open the first available keyboard device (it might need to be adjusted)
        device = InputDevice('/dev/input/event0')  # Adjust the path if needed

        # Run the event loop for keypresses
        while True:
            if not handle_input(device):
                break  # Exit the loop if ESC is pressed

            # Control the car's movement and servo angles
            if velocity != 0:
                if velocity > 0:
                    px.forward(velocity)
                else:
                    px.backward(abs(velocity))
            else:
                px.forward(0)

            # Set the steering and camera based on the current turn and tilt/pan angles
            px.set_dir_servo_angle(turn)
            px.set_cam_tilt_angle(tilt_angle)
            px.set_cam_pan_angle(pan_angle)

            sleep(0.1)  # Add a small delay to avoid overwhelming the system

    finally:
        # Ensure everything is reset when the program ends
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)
        px.set_dir_servo_angle(0)
        px.stop()
        sleep(0.2)

