import RPi.GPIO as GPIO
import time
import serial
from inputs import get_gamepad

# GPIO pin configuration
GPIO.setmode(GPIO.BCM)
GPIO_PIN_20 = 20  # GPIO number for the first stepper motor
GPIO_PIN_16 = 16  # GPIO number for the other stepper motor
GPIO_PIN_23 = 23  # GPIO number for the right joystick
GPIO_PIN_24 = 24  # GPIO number for the other right joystick

# Configure GPIO pins as output
GPIO.setup(GPIO_PIN_20, GPIO.OUT)
GPIO.setup(GPIO_PIN_16, GPIO.OUT)
GPIO.setup(GPIO_PIN_23, GPIO.OUT)
GPIO.setup(GPIO_PIN_24, GPIO.OUT)

# Range of motion for servo motors
servo_min_angle = 0
servo_max_angle = 180

# Set the port and baud rate for the DC motor
ser = serial.Serial('/dev/ttyS0', 9600)

# Function to send commands to the DC motor
def send_motor_command(direction):
    ser.write(str(direction).encode())
    time.sleep(0.1)  # Small delay to allow the motor to respond

# Function to map values
def map_value(value, from_low, from_high, to_low, to_high):
    return int((value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low)

try:
    angle_servo1 = 90  # Initial angle of the first servo motor
    angle_servo2 = 90  # Initial angle of the second servo motor
    motor_direction = 0  # State of the DC motor

    while True:
        events = get_gamepad()
        
        for event in events:
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_Y':
                    # Left joystick controls the first servo motor (forward and backward)
                    angle_servo1 = map_value(event.state, -32768, 32767, servo_max_angle, servo_min_angle)
                    print(f"Servo1 Angle: {angle_servo1}")

                    if angle_servo1 > 135:
                        GPIO.output(GPIO_PIN_20, GPIO.HIGH)
                    elif angle_servo1 < 60:
                        GPIO.output(GPIO_PIN_16, GPIO.HIGH)
                    else:
                        GPIO.output(GPIO_PIN_20, GPIO.LOW)
                        GPIO.output(GPIO_PIN_16, GPIO.LOW)
                
                elif event.code == 'ABS_RX':
                    # Left joystick controls the second servo motor (left and right)
                    angle_servo2 = map_value(event.state, -32768, 32767, servo_min_angle, servo_max_angle)
                    print(f"Servo2 Angle: {angle_servo2}")

                    if angle_servo2 > 135:
                        GPIO.output(GPIO_PIN_23, GPIO.HIGH)
                    elif angle_servo2 < 60:
                        GPIO.output(GPIO_PIN_24, GPIO.HIGH)
                    else:
                        GPIO.output(GPIO_PIN_23, GPIO.LOW)
                        GPIO.output(GPIO_PIN_24, GPIO.LOW)

            elif event.ev_type == 'Key':
                if event.code == 'BTN_TR':
                    if event.state == 1:
                        new_direction = 1  # Clockwise direction
                        print("Rotating clockwise.")
                    else:
                        new_direction = 0
                elif event.code == 'BTN_TL':
                    if event.state == 1:
                        new_direction = 2  # Counterclockwise direction
                        print("Rotating counterclockwise.")
                    else:
                        new_direction = 0

                if new_direction != motor_direction:
                    send_motor_command(new_direction)
                    motor_direction = new_direction

        time.sleep(0.1)

except KeyboardInterrupt:
    if motor_direction != 0:
        send_motor_command(0)
    ser.close()
finally:
    GPIO.cleanup()
