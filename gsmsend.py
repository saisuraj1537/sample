import serial
import time

# Configure the serial connection (adjust the port and baudrate as needed)
ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)

def send_sms(phone_number, message):
    try:
        # Set SMS mode to text
        ser.write(b'AT+CMGF=1\r\n')
        time.sleep(1)
        response = ser.read_all().decode('utf-8', errors='replace')
        print(f"Response after setting text mode: {response}")

        # Set recipient phone number
        ser.write(f'AT+CMGS="{phone_number}"\r\n'.encode('utf-8'))
        time.sleep(1)
        response = ser.read_all().decode('utf-8', errors='replace')
        print(f"Response after setting phone number: {response}")

        # Send the message
        ser.write(f'{message}\r\n'.encode('utf-8'))
        ser.write(bytes([26]))  # CTRL+Z to send the message
        time.sleep(5)

        # Read response from GSM module
        response = ser.read_all().decode('utf-8', errors='replace')
        print(f"Response after sending message: {response}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        ser.close()

# Example usage
phone_number = '6309038588'  # Replace with actual phone number
message = 'Hello from Raspberry Pi!'
send_sms(phone_number, message)
