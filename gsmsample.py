import serial
import time

# Adjust serial port and baud rate as needed
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)  # Ensure this is the correct port

def send_at_command(command, delay=1):
    ser.write((command + '\r').encode())
    time.sleep(delay)
    response = ser.read_all()
    print("Raw response:", response)  # Print raw bytes to debug
    try:
        response = response.decode('utf-8')
    except UnicodeDecodeError:
        response = response.decode('latin-1')  # Try another encoding if needed
    return response

def read_sms():
    # Set SMS mode to text
    response = send_at_command('AT+CMGF=1')
    print("SMS Mode Response:", response)
    
    # Read all messages
    response = send_at_command('AT+CMGL="ALL"')
    print("SMS Messages:", response)

try:
    while True:
        read_sms()
        time.sleep(10)  # Adjust the delay as needed
except KeyboardInterrupt:
    print("Program terminated")
finally:
    ser.close()
