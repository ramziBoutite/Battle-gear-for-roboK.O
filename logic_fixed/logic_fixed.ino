// Ultrasonic Sensor Pins
#define TRIGGER_PIN1  52   // Trigger pin for sensor 1
#define ECHO_PIN1     50   // Echo pin for sensor 1

#define TRIGGER_PIN2  51   // Trigger pin for sensor 2
#define ECHO_PIN2     53   // Echo pin for sensor 2

#define TRIGGER_PIN3  2   // Trigger pin for sensor 3
#define ECHO_PIN3     3   // Echo pin for sensor 3
//
long duration1, duration2, duration3;
float distance1, distance2, distance3;

// IR Sensor Pins
#define IR1 22
#define IR2 23
#define IR3 26
#define IR4 30
// Motor Control Pins (H-Bridge)
#define ENA 5 // Speed control for Motor A (PWM)
#define IN1 6 // Direction control for Motor A
#define IN2 7 // Direction control for Motor A
#define ENB 11 // Speed control for Motor B (PWM)
#define IN3 10 // Direction control for Motor B
#define IN4 9 // Direction control for Motor B

bool ir1Detected = 0;
bool ir2Detected = 0;
bool ir3Detected = 0;
bool ir4Detected = 0;


int detection_distance = 0;
void setup() {
  delay(5000);
  // Motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set the trigger pins as OUTPUT
  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(TRIGGER_PIN2, OUTPUT);
  pinMode(TRIGGER_PIN3, OUTPUT);

  // Set the echo pins as INPUT
  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(ECHO_PIN3, INPUT);

  // IR sensor pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  //Serial.begin(9600);
}

void loop() {

  //read and debug sensors funcion and processing data
  //read from IR
  ir1Detected = digitalRead(IR1);
  ir2Detected = digitalRead(IR2);
  ir3Detected = digitalRead(IR3);
  ir4Detected = digitalRead(IR4);

  //decision taking
  if (ir4Detected &&ir2Detected && ir3Detected && ir1Detected)//0000
  {
    USreading();
    Serial.println(distance2);
    if (distance2 < detection_distance) {
      //forward
      moveMotorA(255);//right
      moveMotorB(255);//left
      //Serial.println("push  ");
    }
   
    else if ( (distance2 > detection_distance)  ) {
      moveMotorA(255);
      moveMotorB(-155);
      //Serial.println("search ");
     // delay(300);
    }


  }
  if (ir4Detected && ir2Detected && ir3Detected && !ir1Detected)//0001
  {
    moveMotorA(-255);
    moveMotorB(-255);
    //Serial.println("going back");
    delay(300);
  }
  if (ir4Detected && ir2Detected && !ir3Detected && ir1Detected)//0010
  {
    moveMotorA(-255);
    moveMotorB(-255);
    //Serial.println("going back");
    delay(300);
  }
  if (ir4Detected && ir2Detected && !ir3Detected && !ir1Detected)//0011
  {
    moveMotorA(-255);
    moveMotorB(-255);
    // Serial.println("going back");
    delay(300);
  }
  if (ir4Detected && !ir2Detected && ir3Detected && ir1Detected)//0100
  {
    //drible back and push
    moveMotorA(255);
    moveMotorB(0);
    //Serial.println("going dribling");
    delay(300);
  }
  if (ir4Detected && !ir2Detected && ir3Detected && !ir1Detected)//0101
  {
    moveMotorA(255);
    moveMotorB(255);
    //Serial.println("going dribling forward ");
    delay(300);
  }
  if (ir4Detected && !ir2Detected && !ir3Detected && ir1Detected)//0110
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }
  if (ir4Detected && !ir2Detected && !ir3Detected && !ir1Detected)//0111
  {
      moveMotorA(0);
      moveMotorB(0);
     // Serial.println("GG"); //gg bro
  }
    if (!ir4Detected && ir2Detected && ir3Detected && ir1Detected)//1000
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }

    if (!ir4Detected && ir2Detected &&  ir3Detected && !ir1Detected)//1001
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }
      if (!ir4Detected && ir2Detected && !ir3Detected && ir1Detected)//1010
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }
      if (!ir4Detected && ir2Detected && !ir3Detected && !ir1Detected)//1011
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }
      if (!ir4Detected && !ir2Detected && ir3Detected && ir1Detected)//1100
  {
    moveMotorA(255);
    moveMotorB(255);
    // Serial.println("going dribling forward ");
    delay(300);
  }


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
void USreading() {

  // Measure distance for sensor 2
  digitalWrite(TRIGGER_PIN2, LOW);
  delayMicroseconds(2);  // Wait for a stable LOW signal
  digitalWrite(TRIGGER_PIN2, HIGH);
  delayMicroseconds(10);  // Send a 10us pulse
  digitalWrite(TRIGGER_PIN2, LOW);

  duration2 = pulseIn(ECHO_PIN2, HIGH, 5000); // Measure the time for echo
  distance2 = (duration2 == 0) ? 100 : constrain(duration2 * 0.0344 / 2, 0, 100);  // Convert time to distance (cm)

}
