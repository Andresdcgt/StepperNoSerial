#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>

const int servo1Pin = 8;
const int servo2Pin = 9;
const int stepsPerRevolution = 800;

bool motorRunning = false;
int motorDirection = 0;  // 0: Stopped, 1: Clockwise, -1: Counterclockwise

Stepper myStepper(stepsPerRevolution, 10, 11, 12, 13);
Servo servo1;
Servo servo2;

enum RobotState {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

volatile RobotState currentState = STOP;

void adjustServo(Servo &servo, int targetAngle) {
    int currentAngle = servo.read();
    int step = (currentAngle < targetAngle) ? 1 : -1;

    while (currentAngle != targetAngle) {
        servo.write(currentAngle);
        delay(10);  // This delay determines the speed of adjustment
        currentAngle += step;
    }
}

void stopMovement() {
  adjustServo(servo1, 90);
  adjustServo(servo2, 90);
}

void moveForward() {
  adjustServo(servo1, 180);
  adjustServo(servo2, 0);
}

void moveBackward() {
  adjustServo(servo1, 0);
  adjustServo(servo2, 180);
}

void turnLeft() {
  adjustServo(servo1, 0);
  adjustServo(servo2, 90);
}

void turnRight() {
  adjustServo(servo1, 90);
  adjustServo(servo2, 180);
}

void gpio20Interrupt() {
    if (digitalRead(2) == HIGH && currentState != FORWARD) {
        currentState = FORWARD;
        moveForward();
    } else if(currentState != STOP) {
        currentState = STOP;
        stopMovement();
    }
}

void gpio16Interrupt() {
  if (digitalRead(3) == HIGH && currentState != BACKWARD) {
    currentState = BACKWARD;
    moveBackward();
  } else if(currentState != STOP) {
        currentState = STOP;
        stopMovement();
    }
}

void gpio23Interrupt() {
  if (digitalRead(18) == HIGH && currentState != RIGHT) {
    currentState = RIGHT;
    turnRight();
  } else if(currentState != STOP) {
        currentState = STOP;
        stopMovement();
    }
}

void gpio24Interrupt() {
  if (digitalRead(19) == HIGH && currentState != LEFT) {
    currentState = LEFT;
    turnLeft();
  }  else if(currentState != STOP) {
        currentState = STOP;
        stopMovement();
    }
}

void setup() {
  // Configure input pins with pull-up resistors
  pinMode(2, INPUT_PULLUP); // Pin 2 for GPIO 20
  pinMode(3, INPUT_PULLUP); // Pin 3 for GPIO 16
  pinMode(18, INPUT_PULLUP); // Pin 18 for GPIO 23
  pinMode(19, INPUT_PULLUP); // Pin 19 for GPIO 24 

  Serial.begin(9600);
  myStepper.setSpeed(15); // RPM
  Serial.println("Arduino ready");

  
  // Attach interrupts to the pins
  attachInterrupt(digitalPinToInterrupt(2), gpio20Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), gpio16Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), gpio23Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), gpio24Interrupt, CHANGE);

  // Initialize servos in the neutral position (90 degrees) at startup
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(90);
  servo2.write(90);
  stopMovement();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '1' && motorDirection != 1) {
      motorDirection = 1;
      motorRunning = true;
      Serial.println("Starting motor clockwise...");
    } else if (command == '2' && motorDirection != -1) {
      motorDirection = -1;
      motorRunning = true;
      Serial.println("Starting motor counterclockwise...");
    } else if (command == '0') {
      motorDirection = 0;
      motorRunning = false;
      myStepper.step(0);  // Stop the motor
      Serial.println("Stopping motor...");
    }

    if (motorRunning) {
      myStepper.step(motorDirection * 200);
      delay(1000);  // Add a delay to control the speed
    }
  }
}
