#include <NewPing.h>

// Ultrasonic Sensor Pins
#define TRIG1 2
#define ECHO1 3
#define TRIG2 4
#define ECHO2 5
#define TRIG3 6
#define ECHO3 7

// IR Sensor Pins
#define IR1 8
#define IR2 9

// Motor Control Pins (H-Bridge)
#define ENA 10 // Speed control for Motor A
#define IN1 11 // Direction control for Motor A
#define IN2 12 // Direction control for Motor A
#define ENB 13 // Speed control for Motor B
#define IN3 A0 // Direction control for Motor B
#define IN4 A1 // Direction control for Motor B

// Ultrasonic Sensors Setup
NewPing sonar1(TRIG1, ECHO1, 200); // Sensor 1, max distance 200 cm
NewPing sonar2(TRIG2, ECHO2, 200); // Sensor 2
NewPing sonar3(TRIG3, ECHO3, 200); // Sensor 3

void setup() {
  // Motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR sensor pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Read ultrasonic sensors
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  unsigned int distance3 = sonar3.ping_cm();

  // Read IR sensors
  bool ir1Detected = digitalRead(IR1);
  bool ir2Detected = digitalRead(IR2);

  // Debugging information
  Serial.print("Ultrasonic: ");
  Serial.print(distance1);
  Serial.print("cm, ");
  Serial.print(distance2);
  Serial.print("cm, ");
  Serial.print(distance3);
  Serial.println("cm");

  Serial.print("IR Sensors: IR1=");
  Serial.print(ir1Detected);
  Serial.print(", IR2=");
  Serial.println(ir2Detected);


}

// Motor Control Functions
void moveMotorA(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void stopMotorA() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void moveMotorB(int speed) {
  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotorB() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
