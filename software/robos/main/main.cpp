#include <Arduino.h>
#include "driver/adc.h"
#include <Wire.h>
#include "MPU6050.h"

#define ENA  25  // PWM Motor A
#define IN1  26
#define IN2  27
#define ENB  33  // PWM Motor B
#define IN3  32
#define IN4  35

#define ENCODER_A1 34
#define ENCODER_A2 39
#define ENCODER_B1 36
#define ENCODER_B2 4

#define BATTERY_PIN  13 // ADC1

MPU6050 accelgyro;

volatile int encoderA_count = 0;
volatile int encoderB_count = 0;

void IRAM_ATTR encoderA_ISR() { encoderA_count++; }
void IRAM_ATTR encoderB_ISR() { encoderB_count++; }

void setupMotors() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void moveMotor(int pwmA, int pwmB) {
  digitalWrite(IN1, pwmA >= 0); digitalWrite(IN2, pwmA < 0);
  digitalWrite(IN3, pwmB >= 0); digitalWrite(IN4, pwmB < 0);
  analogWrite(ENA, abs(pwmA));
  analogWrite(ENB, abs(pwmB));
}

void setupEncoders() {
  pinMode(ENCODER_A1, INPUT); pinMode(ENCODER_B1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderB_ISR, RISING);
}

void setupIMU() {
  Wire.begin();
  accelgyro.initialize();
  if (!accelgyro.testConnection()) {
    Serial.println("MPU6050 não conectado!");
  }
}

float readBattery() {
  int raw = analogRead(BATTERY_PIN);
  return (raw / 4095.0) * 3.3 * 2.0; // Supondo divisor de tensão 1:2
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupEncoders();
  setupIMU();
}

void loop() {
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);

  float battery = readBattery();

  Serial.printf("EncA: %d, EncB: %d | AX: %d, AY: %d, AZ: %d | Bat: %.2fV\n",
                encoderA_count, encoderB_count, ax, ay, az, battery);

  delay(200);
}
