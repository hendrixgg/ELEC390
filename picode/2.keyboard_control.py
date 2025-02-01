from picarx import Picarx
from time import sleep
from pynput import keyboard

manual = '''
Press keys on keyboard to control PiCar-X!
    w: Forward
    a: Turn left
    s: Backward
    d: Turn right
    i: Head up
    k: Head down
    j: Turn head left
    l: Turn head right
    ctrl+c: Press twice to exit the program
'''

def show_info():
    print("\033[H\033[J",end='')  # clear terminal windows
    print(manual)


if __name__ == "__main__":
    try:
        pan_angle = 0
        tilt_angle = 0
        px = Picarx()
        show_info()
        while True:
            if(keyboard.is_pressed('w')):
                px.forward(10)
            elif(keyboard.is_pressed('s')):
                px.backward(10)
            else:
                px.forward(0)
            if(keyboard.is_pressed('d')):
                px.set_dir_servo_angle(30)
            if(keyboard.is_pressed('a')):
                px.set_dir_servo_angle(-30)
            else:
                px.set_dir_servo_angle(0)
            show_info()                     

    finally:
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)


