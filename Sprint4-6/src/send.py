import serial
import time
import os
import argparse

# Correct port adjusted for Raspberry Pi 5 and CH340 chips
USB_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
LINE_TIMEOUT_S = 180
MOTION_TIMEOUT_S = 180
HOME_TIMEOUT_S = 90


def wait_for_ok(serial_conn, timeout_s=LINE_TIMEOUT_S):
    start = time.time()
    while (time.time() - start) < timeout_s:
        line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        lower = line.lower()
        if lower == 'ok' or lower.startswith('ok ') or 'ok' in lower:
            return True
        if 'error' in lower or 'alarm' in lower:
            print(f"Machine error: {line}")
            return False

        # Ignore common non-terminal messages while waiting for completion.
        if lower.startswith('<') or lower.startswith('[msg:'):
            continue

        print(f" Machine info: {line}")
    print(" Timed out waiting for the machine response.")
    return False


def timeout_for_line(line: str) -> int:
    code = line.split(";", 1)[0].strip().upper()
    if code.startswith(("G0", "G1", "G2", "G3")):
        return MOTION_TIMEOUT_S
    return LINE_TIMEOUT_S


def send_gcode(file_path, do_home=False):
    # 1. Check whether the G-code file exists
    if not os.path.exists(file_path):
        print(f"Error: File not found {file_path}")
        return 1

    try:
        print(f"Connecting to plotter: {USB_PORT}...")
        # 2. Establish the serial connection
        s = serial.Serial(USB_PORT, BAUD_RATE, timeout=1, write_timeout=2)
        
        # 3. Wake up the Grbl firmware (send a newline and wait for initialization)
        s.write(b"\r\n\r\n")
        time.sleep(2)   # Give the machine a moment to respond
        s.reset_input_buffer()

        if do_home:
            print("Running homing $H ...")
            s.write(b"$H\n")
            if not wait_for_ok(s, timeout_s=HOME_TIMEOUT_S):
                print("Homing failed, stopping transmission.")
                s.close()
                return 2

        print(f"Starting to send commands: {file_path}")
        sent = 0
        
        with open(file_path, 'r') as f:
            for line in f:
                l = line.strip()
                # Skip blank lines and comments
                if not l or l.startswith(';'):
                    continue
                
                # Send the command to the machine
                s.write((l + '\n').encode('utf-8'))
                sent += 1
                
                # Wait for 'ok' before sending the next line to avoid buffer overflow
                if not wait_for_ok(s, timeout_s=timeout_for_line(l)):
                    print(f" Command transmission failed, aborting job. Failed line: {sent} -> {l}")
                    s.close()
                    return 3

        print("\n✨ Plotting job completed!")
        s.close()
        return 0
        
    except serial.SerialException as e:
        print(f" Serial error: {e}")
        print("Please check: 1. the plotter is powered on 2. the USB cable is firmly connected 3. you ran sudo chmod 666 /dev/ttyUSB0")
        return 4
    except Exception as e:
        print(f"An unknown error occurred: {e}")
        return 5

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send G-code to GRBL over serial")
    parser.add_argument("gcode_file", nargs="?", default="output/draw.ngc")
    parser.add_argument("--home", action="store_true", help="Run $H before drawing")
    args = parser.parse_args()
    raise SystemExit(send_gcode(args.gcode_file, do_home=args.home))
