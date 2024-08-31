###########DEPENDENCIES################
import time
import socket
# import exceptions
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np

#######VARIABLES####################
## Aruco
id_to_find = 143
marker_size = 50  # cm
takeoff_height = 8
velocity = 0.5
lat = 16.565730
long = 81.521693

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_4X4_250)
parameters = aruco.DetectorParameters_create()
##

## Camera
horizontal_res = 640
vertical_res = 480

cap = cv2.VideoCapture(0)  # Initialize the webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

horizontal_fov = 62.2 * (math.pi / 180)  # Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    # Pi cam V1: 41.41 V2: 48.8

# calib_path = "/home/pi/video2calibration/calibrationFiles/"
calib_path = "/home/dronepi/opencv-4.5.1/how_do_drones_work/opencv"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')
##

## Counters and script triggers
found_count = 0
notfound_count = 0

first_run = 0  # Used to set initial time of function to determine FPS
start_time = 0
end_time = 0
script_mode = 1  # 1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land = 0  # 1 to trigger landing

manualArm = False  # If True, arming from RC controller, If False, arming from this script. 
#########FUNCTIONS#################

def connectMyCopter():
    parser =  argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string ,baud=baud_rate ,wait_ready=True)
    return  vehicle

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km ) *1000

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
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(3)



    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_location(to_lat, to_long):

    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to locaton (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat ,to_lon ,to_alt)
    vehicle.simple_goto(to_pont, groundspeed=3)

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

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def lander():
    global first_run, notfound_count, found_count, marker_size, start_time
    if first_run == 0:
        print("First run of lander!!")
        first_run = 1
        start_time = time.time()
        
    ret, frame = cap.read()  # Read frame from camera
    frame = cv2.resize(frame, (horizontal_res, vertical_res))
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    ids = ''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    
    if vehicle.mode != 'LAND':
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode != 'LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
            
    try:
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=cameraMatrix, distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
            
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
            y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
    
            x_avg = x_sum * 0.25
            y_avg = y_sum * 0.25
            
            x_ang = (x_avg - horizontal_res * 0.5) * (horizontal_fov / horizontal_res)
            y_ang = (y_avg - vertical_res * 0.5) * (vertical_fov / vertical_res)
            
            if vehicle.mode != 'LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode != 'LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(x_ang, y_ang)
            else:
                send_land_message(x_ang, y_ang)
                pass
            print("X CENTER PIXEL: " + str(x_avg) + " Y CENTER PIXEL: " + str(y_avg))
            print("FOUND COUNT: " + str(found_count) + " NOTFOUND COUNT: " + str(notfound_count))
            print("MARKER POSITION: x=" + x + " y= " + y + " z=" + z)
            found_count += 1
            print("")
        else:
            notfound_count += 1
    except Exception as e:
        print('Target likely not found. Error: ' + str(e))
        notfound_count += 1

######################################################

#######################MAIN###########################

######################################################

vehicle = connectMyCopter()

##
##SETUP PARAMETERS TO ENABLE PRECISION LANDING
##
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1  # 1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0  # 0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 20  # Descent speed of 30cm/s

if script_mode == 1:
    arm_and_takeoff(takeoff_height)
    
    time.sleep(3)
    
    goto_location(lat, long)
    
    print(str(time.time()))
    # send_local_ned_velocity(velocity, velocity, 0)  # Offset drone from target
    time.sleep(1)
    ready_to_land = 1
elif script_mode == 2:
    while vehicle.mode != 'GUIDED':
        time.sleep(1)
        print("Waiting for manual change from mode " + str(vehicle.mode) + " to GUIDED")
    ready_to_land = 1

if ready_to_land == 1:
    while vehicle.armed:
        lander()
    end_time = time.time()
    total_time = end_time - start_time
    total_time = abs(int(total_time))

    total_count = found_count + notfound_count
    freq_lander = total_count / total_time
    print("Total iterations: " + str(total_count))
    print("Total seconds: " + str(total_time))
    print("------------------")
    print("lander function had frequency of: " + str(freq_lander))
    print("------------------")
    print("Vehicle has landed")
    print("------------------")

cap.release()  # Release the webcam