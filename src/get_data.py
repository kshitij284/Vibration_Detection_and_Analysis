import serial
from datetime import datetime

port = '/dev/tty.usbmodem113203'  
baud = 9600
filename = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

with serial.Serial(port, baud, timeout=1) as ser, open(filename, 'w') as f:
    print(f"Logging to {filename}")
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            f.write(line + '\n')


# import serial
# from datetime import datetime
# import keyboard  # install via: pip install keyboard

# port = '/dev/tty.usbmodem113103'  
# baud = 9600
# filename = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

# paused = False  # Start in recording mode

# with serial.Serial(port, baud, timeout=1) as ser, open(filename, 'w') as f:
#     print(f"Logging to {filename}")
#     print("Press SPACE to pause/resume recording. Press ESC to stop.")
    
#     while True:
#         if keyboard.is_pressed('esc'):
#             print("\nExiting logging.")
#             break

#         if keyboard.is_pressed('space'):
#             paused = not paused
#             print("Paused" if paused else "Resumed")
#             while keyboard.is_pressed('space'):
#                 pass  # wait for key release to prevent toggling too fast

#         line = ser.readline().decode('utf-8', errors='ignore').strip()
#         if line:
#             print(line)
#             if not paused:
#                 f.write(line + '\n')
