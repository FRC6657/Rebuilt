import serial
import pyvjoy
import time
from networktables import NetworkTables

KEY_MAP = {
    '1': 1, '2': 2, '3': 3, 'A': 4,
    '4': 5, '5': 6, '6': 7, 'B': 8,
    '7': 9, '8': 10, '9': 11, 'C': 12,
    '*': 13, '0': 14, '#': 15, 'D': 16
}

# vJoy setup
print("Setting up vJoy...")
j = pyvjoy.VJoyDevice(1)
print("vJoy ready")

# Serial setup
COM_PORT = 'COM19'
print(f"Opening {COM_PORT}...")
ser = serial.Serial(COM_PORT, 9600, timeout=0.1)
time.sleep(2)
print("Serial ready")

# NetworkTables setup
ROBOT_IP = 'localhost'
print(f"Connecting to robot at {ROBOT_IP}...")
NetworkTables.initialize(server=ROBOT_IP)
time.sleep(1)
sd = NetworkTables.getTable('SmartDashboard')
print("NetworkTables ready")

# Track state
last_mode = ""
last_battery = 0
last_tracking = False

def send_lcd(line1, line2):
    """Send LCD update"""
    msg = f"LCD:{line1[:16]}|{line2[:16]}\n"
    ser.write(msg.encode())
    print(f"LCD: {line1[:16]} | {line2[:16]}")

send_lcd("Bridge Ready", "Connecting...")
time.sleep(1)

print("\nRunning...\n")

last_lcd_update = 0

try:
    while True:
        # Read keypad
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if ':' in line:
                    key, state = line.split(':')
                    if key in KEY_MAP:
                        j.set_button(KEY_MAP[key], int(state))
                        print(f"Button {key}: {state}")
            except:
                pass
        
        # Update LCD every 1 second
        now = time.time()
        if now - last_lcd_update > 1.0:
            try:
                mode = sd.getString('RobotMode', 'DISCONNECTED')
                battery = sd.getNumber('Battery Voltage', 0.0)
                tracking = sd.getBoolean('isTracking', False)
                
                # Only update if something changed
                if mode != last_mode or abs(battery - last_battery) > 0.1 or tracking != last_tracking:
                    line1 = f"{mode}{'[T]' if tracking else ''}"
                    line2 = f"Bat: {battery:.1f}V"
                    send_lcd(line1, line2)
                    
                    last_mode = mode
                    last_battery = battery
                    last_tracking = tracking
                
            except Exception as e:
                print(f"Error: {e}")
            
            last_lcd_update = now
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nShutdown...")
    send_lcd("Goodbye!", "")
    time.sleep(1)
    for i in range(1, 17):
        j.set_button(i, 0)
    ser.close()