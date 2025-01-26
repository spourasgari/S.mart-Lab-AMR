// Arduino Code
// Declare motor pins
int RMotorOn = 2;
int RMotorDir = 3;
int RMotorTorque = 4;
int MotorSpeed = 5;
int LMotorOn = 8;
int LMotorDir = 9;
int LMotorTorque = 10;

// Encoder pins
int RMotorEncoder = 20; // CHECK
int LMotorEncoder = 21; // CHECK

volatile int RMotorCount = 0; // Right motor pulse count
volatile int LMotorCount = 0; // Left motor pulse count

float speedDutyCycle = 0.25; // Default speed
unsigned long movementStartTime = 0; // Tracks the start time of a movement
const unsigned long movementDuration = 500; // Duration to move (1000 ms)

bool isMoving = false; // Tracks if the motor is currently moving

void setup() {

  Serial.begin(9600);
  Serial.println("Motor Control System Initialized");

  // Set up motor control pins
  pinMode(RMotorOn, OUTPUT);
  pinMode(RMotorDir, OUTPUT);
  pinMode(RMotorTorque, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(LMotorOn, OUTPUT);
  pinMode(LMotorDir, OUTPUT);
  pinMode(LMotorTorque, OUTPUT);

  // Set up encoder pins as inputs with interrupts
  pinMode(RMotorEncoder, INPUT);
  pinMode(LMotorEncoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(RMotorEncoder), countRMotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMotorEncoder), countLMotor, FALLING);

  //Set initial speed to zero
  analogWrite(MotorSpeed, 0);

  //Enable motors
  digitalWrite(RMotorOn, HIGH);
  digitalWrite(LMotorOn, HIGH);

  //Define initial direction
  digitalWrite(RMotorDir, HIGH);
  digitalWrite(LMotorDir, HIGH);

  //Torque
  analogWrite(RMotorTorque, 0);
  analogWrite(LMotorTorque, 0);
}

void loop() {


  // Check for incoming serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Serial.print("Received command: '");
    Serial.print(command);
    Serial.println("'");

    // Trim any whitespace
    command.trim();
    
    handleCommand(command);
  }

  // Send encoder data to the Raspberry Pi every 100 ms
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 100) {
    lastSendTime = millis();
    Serial.print("R:"); Serial.print(RMotorCount); // Send right encoder value
    Serial.print(" L:"); Serial.println(LMotorCount); // Send left encoder value
  }

  // Check if the movement duration has passed and stop the motor if needed
  if (isMoving && (millis() - movementStartTime >= movementDuration)) {
    // Stop the motors
    analogWrite(MotorSpeed, 0);
    isMoving = false;
  }
}

void handleCommand(String command) {

  // More robust parsing
  float values[3] = {0, 0, 0};
  int valueCount = 0;
  
  // Split the string by space
  char* token = strtok(const_cast<char*>(command.c_str()), " ");
  while (token != NULL && valueCount < 3) {
    values[valueCount] = atof(token);
    // Serial.print("Parsed value "); 
    // Serial.print(valueCount);
    // Serial.print(": ");
    // Serial.println(values[valueCount]);
    
    token = strtok(NULL, " ");
    valueCount++;
  }

  // Extract parsed values
  float linear_x = values[0];
  float linear_y = values[1];
  float angular_z = values[2];

  // Debug print
  Serial.print("Parsed: linear_x="); Serial.print(linear_x);
  Serial.print(" linear_y="); Serial.print(linear_y);
  Serial.print(" angular_z="); Serial.println(angular_z);

  // Simplified motor control
  float wheel_base = 0.05525; // meters
  
  // Calculate wheel speeds
  float right_speed = linear_x + (angular_z * wheel_base / 2.0);
  float left_speed = linear_x - (angular_z * wheel_base / 2.0);

  Serial.print("Calculated speeds - Right: "); Serial.print(right_speed);
  Serial.print(" Left: "); Serial.println(left_speed);

  // Determine motor directions
  if (right_speed >= 0) {
    digitalWrite(RMotorDir, HIGH);
  } else {
    digitalWrite(RMotorDir, LOW);
  }

  if (left_speed >= 0) {
    digitalWrite(LMotorDir, LOW); // Reversed due to previous comment
  } else {
    digitalWrite(LMotorDir, HIGH);
  }

  // Convert speed to duty cycle
  float max_speed = 0.5; // Adjust based on your robot's capabilities
  speedDutyCycle = min(abs(right_speed) / max_speed, 1.0);
  
  // Set motor speed
  analogWrite(MotorSpeed, speedDutyCycle * 255);

  // Set movement tracking
  movementStartTime = millis();
  isMoving = true;

  Serial.print("Set duty cycle: "); Serial.println(speedDutyCycle);
}

void countRMotor() {
  // Check motor direction based on RMotorDir pin
  if (digitalRead(RMotorDir) == HIGH) {
    RMotorCount++; // Forward
  } else {
    RMotorCount--; // Backward
  }
}

void countLMotor() {
  // Check motor direction based on LMotorDir pin
  if (digitalRead(LMotorDir) == HIGH) {
    LMotorCount--; // Forward (I guess due to the installation orientation, the motion direction is reveresed!)
  } else {
    LMotorCount++; // Backward
  }
}