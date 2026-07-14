#include <SoftwareSerial.h>

// Motor Control Pins
int PA = 6;
int PB = 9;
int AI1 = 4;
int AI2 = 5;
int BI1 = 7;
int BI2 = 8;

// 8-Channel IR Sensor Array Setup
const byte IrIn[8] = {10, 11, 12, 13, A0, A1, A2, A3};
const int weights[8] = {-45, -30, -15, -5, 5, 15, 30, 45};

// Peripherals
int Buzz = 1;
int BtTx = 2;
int BtRx = 3;
int SwitchPin = A6; 

// Base Operational Parameters
int BaseSpeed = 120;
bool robotRunning = false; 

// PID Constants
const float Kp = 3.5;  
const float Kd = 1.5;  
const float Ki = 0.01; 

// PID Variables
int lastError = 0;
long integral = 0;

// Timing Registers for Non-Blocking Execution
unsigned long lastDebugTime = 0;
unsigned long lastSwitchCheck = 0; // Bypasses analog read overhead

SoftwareSerial bluetooth(BtRx, BtTx); 

void setup() {
  Serial.begin(9600);

  pinMode(PA, OUTPUT);
  pinMode(PB, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(Buzz, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(IrIn[i], INPUT);
  }
  
  bluetooth.begin(9600);
  
  digitalWrite(Buzz, HIGH);
  delay(150);
  digitalWrite(Buzz, LOW);
}

void loop() {
  // --- FAST ANALOG BYPASS LOGIC ---
  // Only execute analogRead once every 50ms (20 times per second)
  if (millis() - lastSwitchCheck > 50) {
    lastSwitchCheck = millis();
    int switchVal = analogRead(SwitchPin);
    
    if (switchVal < 200) {
      robotRunning = !robotRunning; // Toggle run state
      
      digitalWrite(Buzz, HIGH);
      delay(80); // Brief feedback pause
      digitalWrite(Buzz, LOW);
      
      if (robotRunning) {
        integral = 0;
        lastError = 0;
      }
    }
  }

  // Idling safeguard (near zero overhead)
  if (!robotRunning) {
    analogWrite(PA, 0);
    analogWrite(PB, 0);
    if (millis() - lastDebugTime > 250) {
      lastDebugTime = millis();
      Serial.println("STATUS: IDLE - Press A6 Switch to Start");
      bluetooth.println("STATUS: IDLE - Press A6 Switch to Start");
    }
    return; 
  }

  // --- CORE PID LINE TRACKING ENGINE (Runs at Full Speed) ---
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
    analogWrite(PA, 0);
    analogWrite(PB, 0);
    return;
  }

  integral += error;                    
  integral = constrain(integral, -1000, 1000); 
  int derivative = error - lastError;   
  
  int motorAdjustment = (error * Kp) + (integral * Ki) + (derivative * Kd);
  lastError = error;                    

  int leftSpeed = BaseSpeed + motorAdjustment;
  int rightSpeed = BaseSpeed - motorAdjustment;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  MF(leftSpeed);  
  MB(rightSpeed); 

  // Telemetry (Runs non-blocking every 100ms)
  if (millis() - lastDebugTime > 100) {
    lastDebugTime = millis();
    String debugMsg = "RUNNING | Error: " + String(error) + " | Sens: ";
    for(int i=0; i<8; i++) {
      debugMsg += String(digitalRead(IrIn[i]));
    }
    Serial.println(debugMsg);
    bluetooth.println(debugMsg);
  }
}

void MF(int Speed) {
  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  analogWrite(PA, Speed);
}

void MB(int Speed) {
  digitalWrite(BI1, HIGH);
  digitalWrite(BI2, LOW);
  analogWrite(PB, Speed);
}
