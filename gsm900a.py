import serial
import time

# Initialize serial connection (replace with your actual port and settings)
ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
received_messages = []

def send_command(command, wait_time=1):
    ser.write(command.encode() + b'\r\n')
    time.sleep(wait_time)
    response = ser.read_all().decode()
    return response

def setup_sim800l():
    send_command('AT')
    send_command('AT+CMGF=1')  # Set SMS to text mode
    send_command('AT+CNMI=1,2,0,0,0')  # Configure module to send SMS data to serial out

def send_sms(phone_number, message):
    send_command(f'AT+CMGS="{phone_number}"')
    send_command(message + chr(26), wait_time=5)  # chr(26) is the Ctrl+Z character to send the SMS

def read_serial():
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        received_messages.append(line)
        # Print the received message to the console
        print("Received:", line)
            
def main():
    setup_sim800l()
    phone_number = "+916309038588"  # Replace with the target phone number
    message = "Hello, this is a test message."
    print("Message sent")
    
    send_sms(phone_number, message)  # Send SMS
    
    z = 0
    while z == 0:
        read_serial()
        for i in received_messages:
            if "google" in i:  # Check for specific keyword
                z = 1
                break

    print("Terminated after detecting the keyword in a message.")

if __name__ == "__main__":
    main()