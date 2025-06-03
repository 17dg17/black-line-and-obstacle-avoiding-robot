// Motor pins
const int leftMotorSpeedPin = 9;
const int rightMotorSpeedPin = 10;
const int leftMotorPin1 = 5;
const int leftMotorPin2 = 6;
const int rightMotorPin1 = 7;
const int rightMotorPin2 = 8;

// Ultrasonic sensor pins (left, front, right)
const int leftSensorTrigger = A1;
const int leftSensorEcho = A0;
const int frontSensorTrigger = A3;
const int frontSensorEcho = A2;
const int rightSensorTrigger = A5;
const int rightSensorEcho = A4;

// Line sensors pins (left, center, right)
const int leftLineSensor = 2;
const int centerLineSensor = 3;
const int rightLineSensor = 4;

// Variables for timing and distance measurements
long frontSensorDuration = 0;
long leftSensorDuration = 0;
long rightSensorDuration = 0;

long frontDistance = 0;
long leftDistance = 0;
long rightDistance = 0;

// Line sensor states
int leftLineState = 0;
int centerLineState = 0;
int rightLineState = 0;

// Motor speed for turning
int turnSpeed = 50;

// Control flags for obstacle avoidance behavior
int obstacleFlag = 1; // For handling right obstacle after left turn
int frontFirstMoveFlag = 1; // First reverse movement on front obstacle
int lineFirstMoveFlag = 1;  // First reverse movement on line obstacle

void setup() {
  Serial.begin(9600);

  // Setup ultrasonic sensors
  pinMode(frontSensorTrigger, OUTPUT);
  pinMode(frontSensorEcho, INPUT);

  pinMode(leftSensorTrigger, OUTPUT);
  pinMode(leftSensorEcho, INPUT);

  pinMode(rightSensorTrigger, OUTPUT);
  pinMode(rightSensorEcho, INPUT);

  // Setup line sensors
  pinMode(leftLineSensor, INPUT);
  pinMode(centerLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);

  // Setup motor pins
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  delay(500);
}

void loop() {
  // Read line sensors
  leftLineState = digitalRead(leftLineSensor);
  centerLineState = digitalRead(centerLineSensor);
  rightLineState = digitalRead(rightLineSensor);

  Serial.print("Left line sensor: "); Serial.println(leftLineState);
  Serial.print("Center line sensor: "); Serial.println(centerLineState);
  Serial.print("Right line sensor: "); Serial.println(rightLineState);

  // Measure front distance
  frontDistance = measureDistance(frontSensorTrigger, frontSensorEcho);
  Serial.print("Front distance: "); Serial.println(frontDistance);

  // Measure left distance
  leftDistance = measureDistance(leftSensorTrigger, leftSensorEcho);
  Serial.print("Left distance: "); Serial.println(leftDistance);

  // Measure right distance
  rightDistance = measureDistance(rightSensorTrigger, rightSensorEcho);
  Serial.print("Right distance: "); Serial.println(rightDistance);

  Serial.println("-----------------------------------------------");

  // Line sensor based decisions (avoid black line)
  if (centerLineState == LOW && rightLineState == LOW && leftLineState == HIGH) {
    if (lineFirstMoveFlag == 1) {
      stopMotors();
      reverseAndTurn();
      lineFirstMoveFlag = 0;
    }
    turnRight();
  }
  else if (centerLineState == LOW && rightLineState == HIGH && leftLineState == LOW) {
    if (lineFirstMoveFlag == 1) {
      stopMotors();
      reverseAndTurn();
      lineFirstMoveFlag = 0;
    }
    turnLeft();
  }
  else if (centerLineState == LOW && rightLineState == HIGH && leftLineState == HIGH) {
    if (lineFirstMoveFlag == 1) {
      stopMotors();
      reverseAndTurn();
      lineFirstMoveFlag = 0;
    }
    turnLeft();
  }
  else if (centerLineState == HIGH && rightLineState == HIGH && leftLineState == HIGH) {
    if (lineFirstMoveFlag == 1) {
      stopMotors();
      reverseAndTurn();
      lineFirstMoveFlag = 0;
    }
    turnLeft();
  }
  else if (centerLineState == HIGH && rightLineState == LOW && leftLineState == LOW) {
    Serial.println("Stop condition met");
    stopMotors();
    delay(10000);
  }

  if (rightDistance >= 16) {
    obstacleFlag = 1;
  }

  // Obstacle avoidance logic
  if (centerLineState == LOW && rightLineState == LOW && leftLineState == LOW) {
    lineFirstMoveFlag = 1;

    // Obstacle avoidance decisions based on ultrasonic sensor readings
    if (leftDistance <= 10 && rightDistance >= 10 && frontDistance <= 20) {
      if (frontFirstMoveFlag == 1) {
        reverseAndTurn();
        frontFirstMoveFlag = 0;
      }
      turnRight();
    }
    else if (leftDistance >= 10 && rightDistance >= 10 && frontDistance <= 20) {
      if (frontFirstMoveFlag == 1) {
        reverseAndTurn();
        frontFirstMoveFlag = 0;
      }
      turnLeft();
      obstacleFlag = 0;
    }
    else if (rightDistance <= 10 && leftDistance >= 10 && frontDistance <= 20) {
      if (frontFirstMoveFlag == 1) {
        reverseAndTurn();
        frontFirstMoveFlag = 0;
      }
      turnLeft();
    }
    else if (rightDistance <= 10 && leftDistance <= 10 && frontDistance <= 20) {
      reverseAndTurn();
    }

    // Move forward if path is clear
    if (frontDistance > 21) {
      frontFirstMoveFlag = 1;
      moveForward();

      if (rightDistance >= 6 && leftDistance >= 6) {
        moveForward();
      }
      if (leftDistance <= 7) {
        turnRight();
      }
      if (obstacleFlag == 1) {
        if (rightDistance <= 7) {
          turnLeft();
        }
      }
    }
  }
}

// Function to measure distance with ultrasonic sensor
long measureDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 33.2 / 2; // Convert duration to cm
}

// Motor control functions
void turnLeft() {
  analogWrite(leftMotorSpeedPin, 0);
  analogWrite(rightMotorSpeedPin, turnSpeed);

  analogWrite(leftMotorPin1, 255);
  analogWrite(leftMotorPin2, 0);

  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 255);

  Serial.println("Turning left");
}

void turnRight() {
  analogWrite(leftMotorSpeedPin, turnSpeed);
  analogWrite(rightMotorSpeedPin, 0);

  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 255);

  analogWrite(rightMotorPin1, 255);
  analogWrite(rightMotorPin2, 0);

  Serial.println("Turning right");
}

void stopMotors() {
  analogWrite(leftMotorSpeedPin, 0);
  analogWrite(rightMotorSpeedPin, 0);

  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);

  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);

  Serial.println("Motors stopped");
}

void moveForward() {
  analogWrite(leftMotorSpeedPin, 44);
  analogWrite(rightMotorSpeedPin, 42);

  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 255);

  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 255);

  Serial.println("Moving forward");
}

void reverseAndTurn() {
  analogWrite(leftMotorSpeedPin, 55);
  analogWrite(rightMotorSpeedPin, 55);

  analogWrite(leftMotorPin1, 255);
  analogWrite(leftMotorPin2, 0);

  analogWrite(rightMotorPin1, 255);
  analogWrite(rightMotorPin2, 0);

  Serial.println("Reversing");
  delay(600);
}
