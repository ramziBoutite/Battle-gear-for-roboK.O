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


// Motor Control Pins (H-Bridge)
#define ENA 5 // Speed control for Motor A (PWM)
#define IN1 6 // Direction control for Motor A
#define IN2 7 // Direction control for Motor A
#define ENB 11 // Speed control for Motor B (PWM)
#define IN3 10 // Direction control for Motor B
#define IN4 9 // Direction control for Motor B

bool ir1Detected = 0;
bool ir2Detected =0;

void setup() {
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

  Serial.begin(9600);
}

void loop() {

  //read and debug sensors funcion and processing data
  //read from IR
   ir1Detected = digitalRead(IR1);
   ir2Detected = digitalRead(IR2);
  // Read ultrasonic sensors
  // Measure distance for sensor 1
  digitalWrite(TRIGGER_PIN1, LOW);
  delayMicroseconds(2);  // Wait for a stable LOW signal
  digitalWrite(TRIGGER_PIN1, HIGH);
  delayMicroseconds(10);  // Send a 10us pulse
  digitalWrite(TRIGGER_PIN1, LOW);

  duration1 = pulseIn(ECHO_PIN1, HIGH, 6000); // Measure the time for echo
  distance1 = (duration1 == 0) ? 100 : constrain(duration1 * 0.0344 / 2, 0, 100); // Convert time to distance (cm)

  // Measure distance for sensor 2
  digitalWrite(TRIGGER_PIN2, LOW);
  delayMicroseconds(2);  // Wait for a stable LOW signal
  digitalWrite(TRIGGER_PIN2, HIGH);
  delayMicroseconds(10);  // Send a 10us pulse
  digitalWrite(TRIGGER_PIN2, LOW);

  duration2 = pulseIn(ECHO_PIN2, HIGH, 6000); // Measure the time for echo
  distance2 = (duration2 == 0) ? 100 : constrain(duration2 * 0.0344 / 2, 0, 100);  // Convert time to distance (cm)

  // Measure distance for sensor 3
  digitalWrite(TRIGGER_PIN3, LOW);
  delayMicroseconds(2);  // Wait for a stable LOW signal
  digitalWrite(TRIGGER_PIN3, HIGH);
  delayMicroseconds(10);  // Send a 10us pulse
  digitalWrite(TRIGGER_PIN3, LOW);

  duration3 = pulseIn(ECHO_PIN3, HIGH, 6000); // Measure the time for echo
  distance3 = (duration3 == 0) ? 100 : constrain(duration3 * 0.0344 / 2, 0, 100); // Convert time to distance (cm)

  // Print the results
  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print(" cm, ");
  Serial.print("Distance 2: ");
  Serial.print(distance2);
  Serial.print(" cm, ");
  Serial.print("Distance 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Wait a bit before taking the next readings
  //delay(50);

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
  
  ir1Detected = digitalRead(IR1);
  ir2Detected = digitalRead(IR2);
  
  if (ir1Detected && !ir2Detected)
  {
    
    moveMotorA(-255); 
    moveMotorB(-255);
    Serial.println("going back");
    delay(1000);
    
    
  }
 if (!ir1Detected && ir2Detected) {
   
    //drible back and push
    moveMotorA(255);
    Serial.println("going dribling");
    delay(1000);    
  }

  if (!ir1Detected && !ir2Detected) {
   
  stopMotorB();
  stopMotorA();
  }

  if (ir1Detected && ir2Detected) {
   
  stopMotorB();  
  stopMotorA();
  }
  stopMotorB();
  stopMotorA();
  
  
  
    

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
