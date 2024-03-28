const int sensorPin = 13;  // The pin connected to the voltage-controlled switch
const int motorPin1 = 7;  // Control pin 1 for the motor driver
const int motorPin2 = 4;  // Control pin 2 for the motor driver
const int enablePin = 3;  // PWM-capable pin connected to the L293D's enable pin

void setup() {
  // Initialize the serial communication:
  Serial.begin(9600);
  // Set motor control and enable pins as output:
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // Set the sensor pin as input:
  pinMode(sensorPin, INPUT);
}

void loop() {
  // Read the state of the voltage-controlled switch:
  int sensorState = digitalRead(sensorPin);

  if (sensorState == HIGH) {
    // Rain detected - rotate motor clockwise at a slower speed
    Serial.println("Rain detected - rotating motor clockwise at reduced speed");
    rotateMotorClockwise();
  } else {
    // No rain detected - rotate motor anti-clockwise at a slower speed
    Serial.println("No rain detected - rotating motor anti-clockwise at reduced speed");
    rotateMotorAntiClockwise();
  }
  
}

void rotateMotorClockwise() {
  analogWrite(enablePin, 128); // Adjust PWM value for desired speed (0 to 255)
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  // Adjust delay based on the desired number of rotations
  delay(5000); 
  stopMotor(); 

}


void rotateMotorAntiClockwise() {
  analogWrite(enablePin, 128); // Adjust PWM value for desired speed
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  // Adjust delay based on the desired number of rotations
  delay(5000);
  stopMotor();

}

void stopMotor(){
  // Stop the motor after rotating
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  analogWrite(enablePin, 0); // Turn off motor

}

