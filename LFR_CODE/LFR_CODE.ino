#include <SoftwareSerial.h>

// Motor Control Pins
int PA = 6;
int PB = 9;
int AI1 = 4;
int AI2 = 5;
int BI1 = 7;
int BI2 = 8;
// STDBY ALWAYS HIGH

// 8-Channel IR Sensor Array Setup
const byte IrIn[8] = {10, 11, 12, 13, A0, A1, A2, A3};
const int weights[8] = {-45, -30, -15, -5, 5, 15, 30, 45};

// Peripherals
int Buzz = 1; 
int BtTx = 2;
int BtRx = 3;
int SwitchPin = A6; 

// Base Operational Parameters (Marked non-const so your phone can overwrite them!)
int BaseSpeed = 70; 
bool robotRunning = false; 

// PID Tuning Parameters (Marked non-const so your phone can overwrite them!)
float Kp = 2.8;  
float Kd = 1.2;  
float Ki = 0.005; 

// PID Calculation Registers
int lastError = 0;
long integral = 0;

// Timing Registers for Non-Blocking Logic
unsigned long lastDebugTime = 0;
unsigned long lastSwitchCheck = 0; 

// Initialize SoftwareSerial for Bluetooth (RX, TX)
SoftwareSerial bluetooth(BtRx, BtTx); 

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Configure H-Bridge Pins
  pinMode(PA, OUTPUT);
  pinMode(PB, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(Buzz, OUTPUT);

  // Configure IR Array Pins
  for (int i = 0; i < 8; i++) {
    pinMode(IrIn[i], INPUT);
  }
  
  // Startup chime
  digitalWrite(Buzz, HIGH);
  delay(150);
  digitalWrite(Buzz, LOW);
}

void loop() {
  // --- COMMENT THIS SINGLE LINE BELOW TO DISABLE PHONE TUNING LATER ---
  checkBluetoothTuning();

  // --- FAST ANALOG SWITCH POLLING BYPASS (Runs every 50ms) ---
  if (millis() - lastSwitchCheck > 50) {
    lastSwitchCheck = millis();
    int switchVal = analogRead(SwitchPin);
    
    if (switchVal < 200) {
      robotRunning = !robotRunning; 
      
      digitalWrite(Buzz, HIGH);
      delay(80); 
      digitalWrite(Buzz, LOW);
      
      if (robotRunning) {
        integral = 0;
        lastError = 0;
      }
    }
  }

  // Idle Protection Safeguard
  if (!robotRunning) {
    ML(0); 
    MR(0); 
    
    if (millis() - lastDebugTime > 250) {
      lastDebugTime = millis();
      Serial.println("STATUS: IDLE - Press A6 Switch to Start");
      bluetooth.println("STATUS: IDLE - Press A6 Switch to Start");
    }
    return; 
  }

  // --- CORE PID LINE TRACKING ENGINE ---
  long totalWeight = 0;
  int activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    if (digitalRead(IrIn[i]) == HIGH) { 
      totalWeight += weights[i];
      activeSensors++;
    }
  }

  int error = 0;
  if (activeSensors > 0) {
    error = totalWeight / activeSensors; 
  } else {
    return; 
  }

  // Compute Active PID Control Values
  integral += error;                    
  integral = constrain(integral, -500, 500); 
  int derivative = error - lastError;   
  
  int motorAdjustment = (error * Kp) + (integral * Ki) + (derivative * Kd);
  lastError = error; 

  int leftSpeed = BaseSpeed + motorAdjustment;
  int rightSpeed = BaseSpeed - motorAdjustment;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  ML(leftSpeed);  
  MR(rightSpeed); 

  // --- TELEMETRY PERFORMANCE STREAMING (Every 100ms) ---
  if (millis() - lastDebugTime > 100) {
    lastDebugTime = millis();
    
    // Notice that this now prints live Kp, Kd, Ki, and Speed values so you can see changes!
    String debugMsg = "P=" + String(Kp) + " D=" + String(Kd) + " S=" + String(BaseSpeed) + " | Err: " + String(error);
    
    Serial.println(debugMsg);
    bluetooth.println(debugMsg);
  }
}

// ==========================================
// STANDALONE BLUETOOTH TUNING ENGINE
// ==========================================
void checkBluetoothTuning() {
  if (bluetooth.available() > 0) {
    char token = bluetooth.read();       // Read the first character (P, I, D, or S)
    float value = bluetooth.parseFloat(); // Read the numeric float value following it
    
    // Quick confirm double-click noise on the buzzer to acknowledge receipt
    digitalWrite(Buzz, HIGH); delay(20); digitalWrite(Buzz, LOW); delay(20);
    digitalWrite(Buzz, HIGH); delay(20); digitalWrite(Buzz, LOW);

    switch (token) {
      case 'P': case 'p':
        Kp = value;
        bluetooth.print("--> Kp updated to: "); bluetooth.println(Kp);
        break;
      case 'D': case 'd':
        Kd = value;
        bluetooth.print("--> Kd updated to: "); bluetooth.println(Kd);
        break;
      case 'I': case 'i':
        Ki = value;
        bluetooth.print("--> Ki updated to: "); bluetooth.println(Ki);
        break;
      case 'S': case 's':
        BaseSpeed = (int)value;
        bluetooth.print("--> BaseSpeed updated to: "); bluetooth.println(BaseSpeed);
        break;
      default:
        bluetooth.println("ERROR: Invalid command token. Use P, D, I, or S.");
        break;
    }
  }
}

// Logic Controller for Left Drive (Motor A)
void ML(int speed) {
  if (speed >= 0) {
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
  } else {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    speed = -speed; 
  }
  analogWrite(PA, speed);
}

// Logic Controller for Right Drive (Motor B)
void MR(int speed) {
  if (speed >= 0) {
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
  } else {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    speed = -speed; 
  }
  analogWrite(PB, speed);
}
