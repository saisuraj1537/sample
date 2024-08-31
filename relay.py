import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin connected to the relay
relay_pin = 17  # Adjust this to the actual GPIO pin you are using

# Set up the GPIO pin as an output
GPIO.setup(relay_pin, GPIO.OUT)

def turn_relay_on():
    print("Turning relay ON")
    GPIO.output(relay_pin, GPIO.HIGH)

def turn_relay_off():
    print("Turning relay OFF")
    GPIO.output(relay_pin, GPIO.LOW)

try:
    # Main loop
    while True:
        turn_relay_on()
        time.sleep(2)  # Wait for 2 seconds
        turn_relay_off()
        time.sleep(2)  # Wait for 2 seconds

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()
