import serial
import time
import os
import sys

# Correct port adjusted for Raspberry Pi 5 and CH340 chipset
USB_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def send_gcode(file_path):
    # 1. Check that the G-code file exists
    if not os.path.exists(file_path):
        print(f"Error: file not found {file_path}")
        return

    try:
        print(f"Connecting to plotter: {USB_PORT}...")
        # 2. Establish serial connection
        s = serial.Serial(USB_PORT, BAUD_RATE, timeout=1)
        
        # 3. Wake up Grbl firmware (send newlines and wait for init)
        s.write(b"\r\n\r\n")
        time.sleep(2)   # give the machine a moment to respond
        s.reset_input_buffer()

        print(f"Starting to send commands: {file_path}")
        
        with open(file_path, 'r') as f:
            for line in f:
                l = line.strip()
                # skip empty lines or comments
                if not l or l.startswith(';'):
                    continue
                
                # send command to the machine
                s.write((l + '\n').encode('utf-8'))
                
                # wait for the machine to reply 'ok' before sending next line (prevent buffer overflow)
                while True:
                    grbl_out = s.readline().decode('utf-8').strip()
                    if grbl_out == 'ok':
                        break
                    elif 'error' in grbl_out.lower():
                        print(f"Machine error: {grbl_out}")
                        break

        print("\n✨ Plotting task completed!")
        s.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("Please check: 1) is the plotter powered on? 2) is the USB cable firmly connected? 3) have you run: sudo chmod 666 /dev/ttyUSB0")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    # Core: prefer receiving a filename passed from main.py
    if len(sys.argv) > 1:
        gcode_file = sys.argv[1]
    else:
        # If run manually (python3 send.py), default to the most recently generated draw.ngc
        gcode_file = "output/draw.ngc"
    
    send_gcode(gcode_file)
