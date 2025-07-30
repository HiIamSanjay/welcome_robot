// robot_motor_control.ino
// This sketch receives single-character commands from a computer via the
// USB serial port and controls a 2-wheel robot using an L298N motor driver.

// --- Pin Definitions for L298N Motor Driver ---
// Connect these pins from the L298N to the corresponding digital pins on the Arduino.

// Left Motor
const int ENA = 9;  // Enable A (Left Motor Speed). Must be a PWM pin (~).
const int IN1 = 7;  // Input 1 for Left Motor.
const int IN2 = 6;  // Input 2 for Left Motor.

// Right Motor
const int ENB = 10; // Enable B (Right Motor Speed). Must be a PWM pin (~).
const int IN3 = 5;  // Input 3 for Right Motor.
const int IN4 = 4;  // Input 4 for Right Motor.

// --- Motor Speed Configuration ---
// Set the motor speed (0 = off, 255 = full speed).
// A value around 200 is a good starting point to avoid being too fast.
int motorSpeed = 200;

void setup() {
  // Initialize serial communication at 9600 bits per second.
  // This must match the baud rate set in the Python script.
  Serial.begin(9600);
  
  // Set all the motor control pins as outputs.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Start with the motors stopped to be safe.
  stopMotors();
  Serial.println("Arduino is ready. Waiting for commands...");
}

void loop() {
  // Check if there is any data available to read from the serial port.
  if (Serial.available() > 0) {
    // Read the incoming byte (character) from the Python script.
    char command = Serial.read();

    // Print the command to the Serial Monitor for easy debugging.
    Serial.print("Command received: ");
    Serial.println(command);

    // Use a switch statement to execute the correct motor function.
    switch (command) {
      case 'f': // 'f' for "forward"
        goForward();
        break;
      case 'b': // 'b' for "backward"
        goBackward();
        break;
      case 'l': // 'l' for "left"
        turnLeft();
        break;
      case 'r': // 'r' for "right"
        turnRight();
        break;
      case 's': // 's' for "stop"
        stopMotors();
        break;
    }
  }
}

// --- Motor Control Functions ---

void goForward() {
  // Left motor spins forward.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed); // Set speed
  
  // Right motor spins forward.
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeed); // Set speed
}

void goBackward() {
  // Left motor spins backward.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, motorSpeed);

  // Right motor spins backward.
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  // To turn left, spin the left motor backward and the right motor forward.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, motorSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  // To turn right, spin the left motor forward and the right motor backward.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  // To stop, set both inputs for each motor to LOW.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Set speed to 0

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0); // Set speed to 0
}
