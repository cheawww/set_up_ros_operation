

// === Motor driver pins ===
#define INA1 14   // Motor 1
#define INB1 12
#define PWM1 13

#define INA2 27   // Motor 2
#define INB2 26
#define PWM2 25

// === Encoder pins ===
#define ENCODER1_A 34   // Motor 1 Encoder
#define ENCODER1_B 35

#define ENCODER2_A 32   // Motor 2 Encoder
#define ENCODER2_B 33

volatile long encoder1_ticks = 0;
volatile long encoder2_ticks = 0;

void IRAM_ATTR handleEncoder1A() {
  bool A = digitalRead(ENCODER1_A);
  bool B = digitalRead(ENCODER1_B);
  encoder1_ticks += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncoder2A() {
  bool A = digitalRead(ENCODER2_A);
  bool B = digitalRead(ENCODER2_B);
  encoder2_ticks += (A == B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  // Setup motors
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Setup encoders
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), handleEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), handleEncoder2A, CHANGE);

  // Stop motors at startup
  stopMotors();
}

void loop() {
  // Send encoder ticks every 100ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {
    lastSend = millis();

    // Format: ENC L:<ticks> R:<ticks>
    Serial.print("ENC L:");
    Serial.print(encoder1_ticks);
    Serial.print(" R:");
    Serial.println(encoder2_ticks);
  }

  // TODO: Optionally listen to Serial commands like "SET L:100 R:100"
}

// === Motor control helper ===
void setMotor(int inA, int inB, int pwmPin, int speed) {
  bool dir = speed >= 0;
  digitalWrite(inA, dir);
  digitalWrite(inB, !dir);
  analogWrite(pwmPin, abs(speed));
}

void driveMotors(int leftSpeed, int rightSpeed) {
  setMotor(INA1, INB1, PWM1, leftSpeed);
  setMotor(INA2, INB2, PWM2, rightSpeed);
}

void stopMotors() {
  driveMotors(0, 0);
}