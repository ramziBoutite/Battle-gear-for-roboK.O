#include <NewPing.h>



// Ultrasonic Sensor Pins
#define TRIG1 12
#define ECHO1 13
#define TRIG2 14
#define ECHO2 15
#define TRIG3 16
#define ECHO3 17

// IR Sensor Pins
#define IR1 18
#define IR2 19

// Motor Control Pins (H-Bridge)
#define ENA 5 // Speed control for Motor A (PWM)
#define IN1 6 // Direction control for Motor A
#define IN2 7 // Direction control for Motor A
#define ENB 11 // Speed control for Motor B (PWM)
#define IN3 10 // Direction control for Motor B
#define IN4 9 // Direction control for Motor B

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

  //read and debug sensors funcion and processing data
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
  ///////////////////////////////////////////////////////////

  /*
  ///motor test
   
   Serial.println("Testing Motor A Forward");
   moveMotorA(255); // Full speed forward
   Serial.println("Testing Motor B Forward");
   moveMotorB(255);
   delay(2000);     // Run for 2 seconds
   
   Serial.println("Testing Motor A Backward");
   moveMotorA(-255); // Full speed backward
   Serial.println("Testing Motor B Backward");
   moveMotorB(-255); // Full speed backward
   delay(2000);      // Run for 2 seconds
   
   Serial.println("Stopping Motor A");
   stopMotorA();
   Serial.println("Stopping Motor B");
   stopMotorB();
   delay(2000); // Pause for 2 seconds
   */
  //decision taking
  if(ir1Detected)
  {
    //back
  }
  else if (ir2Detected){
    //drible back and push
  }
  /*
  //ultra sonic based decision making
else if()
   
   */

}




// Motor Control Functions
// Function to move Motor A
void moveMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } 
  else {
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
  } 
  else {
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


