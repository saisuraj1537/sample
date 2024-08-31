

# Configure the serial port for communication with the SIM800L module

list = []
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math
import socket
import argparse
import geopy.distance
import serial

#import face_recognition
#import picamera
import numpy as np

relay = 17
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay,GPIO.OUT)
GPIO.setwarnings(False)

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

def turn_on():
    GPIO.output(relay, GPIO.LOW)  # Turn on the relay
    print("Relay turned ON")

def turn_off():
    GPIO.output(relay, GPIO.HIGH)  # Turn off the relay
    print("Relay turned OFF")

turn_off()
time.sleep(1)
turn_on()
time.sleep(1)
turn_off()

#connect to drone
def connectMyCopter():
  parser =  argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()

  connection_string = args.connect
  baud_rate = 57600
  print("\nConnecting to vehicle on: %s" % connection_string)
  vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
  return vehicle


#arm and takeoff to meteres
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    time.sleep(3)

    

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
        
def get_dstance(cord1, cord2):
    #return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km)*1000        
        
def goto_location(to_lat, to_long): 
        
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to locaton (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat,to_lon,to_alt)
    vehicle.simple_goto(to_pont, groundspeed=6)
    
    to_cord = (to_lat, to_lon)
    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)


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

lg = list[-1]
lat1s = lg.index("query=") + 6
lat1e = lg.index("%")
#latitude = float(lg[lat1s:lat1e])
#print(latitude)
longi1s = lg.index("C") + 1
#longitude = float(lg[longi1s:])
#print(longitude)
time.sleep(5)
vehicle = connectMyCopter()
time.sleep(2)
ht = 15
arm_and_takeoff(ht)
turn_on()
#GPIO.output(relay,True)
latitude = float(lg[lat1s:lat1e])
print(latitude)
longitude = float(lg[longi1s:])
print(longitude)
time.sleep(2)
goto_location(latitude,longitude)
#GPIO.output(relay,False)
time.sleep(20)
turn_off()
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
