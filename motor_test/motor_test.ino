// Motor Control Pins (H-Bridge)
#define ENA 10 // Speed control for Motor A (PWM)
#define IN1 11 // Direction control for Motor A
#define IN2 12 // Direction control for Motor A
#define ENB 13 // Speed control for Motor B (PWM)
#define IN3 A0 // Direction control for Motor B
#define IN4 A1 // Direction control for Motor B

void setup() {
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.println("Testing Motor A Forward");
  moveMotorA(255); // Full speed forward
  delay(2000);     // Run for 2 seconds

  Serial.println("Testing Motor A Backward");
  moveMotorA(-255); // Full speed backward
  delay(2000);      // Run for 2 seconds

  Serial.println("Stopping Motor A");
  stopMotorA();
  delay(2000); // Pause for 2 seconds

  Serial.println("Testing Motor B Forward");
  moveMotorB(255); // Full speed forward
  delay(2000);     // Run for 2 seconds

  Serial.println("Testing Motor B Backward");
  moveMotorB(-255); // Full speed backward
  delay(2000);      // Run for 2 seconds

  Serial.println("Stopping Motor B");
  stopMotorB();
  delay(2000); // Pause for 2 seconds
}

// Function to move Motor A
void moveMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed; // Make speed positive for PWM
  }
  analogWrite(ENA, speed);
}

// Function to stop Motor A
void stopMotorA() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// Function to move Motor B
void moveMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed; // Make speed positive for PWM
  }
  analogWrite(ENB, speed);
}

// Function to stop Motor B
void stopMotorB() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

