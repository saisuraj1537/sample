import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep
def servo():
    GPIO.setmode(GPIO.BOARD)

    # Set up pin 11 for PWM
    GPIO.setup(11, GPIO.OUT)
    p1 = GPIO.PWM(11, 50)  # Servo 1 on pin 11
    p1.start(0)

    # Set up pin 13 for PWM
    GPIO.setup(13, GPIO.OUT)
    p2 = GPIO.PWM(13, 50)  # Servo 2 on pin 13
    p2.start(0)

    try:
            # Move servo 1 back and forth
        p1.ChangeDutyCycle(3)
        sleep(5)
        p2.ChangeDutyCycle(3)
        sleep(10)

            # Move servo 2 back and forth
        p2.ChangeDutyCycle(12)
        sleep(3)
        p1.ChangeDutyCycle(9)
        sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        # At the end of the1 program, stop the PWM and clean up
        p1.stop()
        p2.stop()
        GPIO.cleanup()
servo()