#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "HX711.h"
#include <SPI.h>

// ---- PCA9685 (servo driver) ----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---- HX711 (force sensor) ----
#define HX711_DT  3
#define HX711_SCK 2
HX711 scale;
float upper_threshold = 500000.0;   // adjust threshold as needed
float lower_threshold = 0.0;

// ---- SPI ----
volatile byte spiBuffer[12];
volatile bool commandReceived = false;

// ---- Servo channel ----
#define PINKY_CHANNEL  15
#define RING_CHANNEL  14
#define MIDDLE_CHANNEL  13
#define INDEX_CHANNEL  12 
int servoMin = 150;       // Calibrate your servo min pulse
int servoMax = 600;       // Calibrate your servo max pulse
float angleFinger[6];
byte feedback[14];

// ---- Status codes ----
byte status = 0x00; // 0x00 idle, 0x01 moving, 0x02 done, 0x12 halt

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);   // 50 Hz for analog servos

  // HX711 init
  scale.begin(HX711_DT, HX711_SCK);

  // SPI Slave init
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);     // enable SPI
  SPI.attachInterrupt();

  Serial.println("=== Demo Hand Low Level Control Ready ===");
}

ISR(SPI_STC_vect) {
  static int index = 0;
  static int replyIndex = 1;
  byte c = SPDR;        // read received byte
  spiBuffer[index++] = c;
  if (index >= 14) {    // got full packet (6 DOFs × 2 bytes)
    commandReceived = true;
    index = 0;
  }

  SPDR = feedback[replyIndex];      
  replyIndex++;                     
  if (replyIndex >= 14) {            
    replyIndex = 0;
  }
}

void loop() {
  if (commandReceived) {
    commandReceived = false;

    // --- Parse Pinky, Ring, Middle, Index, Thumb DOF1, Thumb DOF2 DOF (bytes 0,1) ---
    for (int i = 0; i < 12; i=i+2) { 
      uint16_t raw = (uint16_t(spiBuffer[i+1]) << 8) | spiBuffer[i]; // LSB first
      angleFinger[i/2] = raw * 0.01; // degrees
      Serial.print(angleFinger[i/2]);
      Serial.print(" ");
    }

    // --- Safety check from HX711 ---
    long val = scale.read_average(5);
    Serial.println(val);
    if (val > upper_threshold && val < lower_threshold) // Adjust as need
    {
      status = 0x12; // Halt
      Serial.println("Sensor triggered! HALT.");
      sendFeedback();
      return;
    }

    // --- Move servo ---
    status = 0x01; // Moving
    sendFeedback();

    int pwmVal_pinky = map((int)angleFinger[0], 0, 180, servoMin, servoMax);
    int pwmVal_ring = map((int)angleFinger[1], 0, 180, servoMin, servoMax);
    int pwmVal_middle = map((int)angleFinger[2], 0, 180, servoMin, servoMax);
    int pwmVal_index = map((int)angleFinger[3], 0, 180, servoMin, servoMax);
    pwm.setPWM(PINKY_CHANNEL, 0, pwmVal_pinky);
    pwm.setPWM(RING_CHANNEL, 0, pwmVal_ring);
    pwm.setPWM(MIDDLE_CHANNEL, 0, pwmVal_middle);
    pwm.setPWM(INDEX_CHANNEL, 0, pwmVal_index);

    delay(1000);   // simulate move time

    status = 0x02; // Done
    sendFeedback();
  } else {
    // --- Keep-alive when idle ---
    status = 0x00;
    sendFeedback();
    delay(100);
  }
}

void sendFeedback() {
  // Echo 12 bytes
  for (int i = 0; i < 12; i++) feedback[i] = spiBuffer[i];
  feedback[12] = 0x00;   // backup
  feedback[13] = status; // status

  // Print feedback
  Serial.print("Feedback: ");
  for (int i = 0; i < 14; i++) {
    Serial.print("0x");
    if (feedback[i] < 16) Serial.print("0");
    Serial.print(feedback[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

