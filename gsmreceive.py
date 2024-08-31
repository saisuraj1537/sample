import serial
import time

# Initialize serial connection (replace with your actual port and settings)
ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
list = []
def send_command(command):
    ser.write(command.encode() + b'\r\n')
    time.sleep(1)
    response = ser.read_all().decode()
    return response

def setup_sim800l():
    send_command('AT')
    send_command('AT+CMGF=1')
    send_command('AT+CNMI=1,2,0,0,0')

def read_serial():
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        list.append(line)
            
        # You can add your SMS handling logic here

def main():
    setup_sim800l()
    z = 0
    while z == 0:
        read_serial()
        print(list)
        for i in list:
            if "google" in i:
                z = 1
                break
main()
print("breaked")