#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>
#include <SparkFun_Qwiic_OLED.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#define MICRO

#if defined(TRANSPARENT)
QwiicTransparentOLED myOLED;
const char *deviceName = "Transparent OLED";
#elif defined(NARROW)
QwiicNarrowOLED myOLED;
const char *deviceName = "Narrow OLED";
#else
QwiicMicroOLED myOLED;
const char *deviceName = "Micro OLED";
#endif

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

int switchPin = 5;
volatile bool buttonTriggered = false;
volatile unsigned long lastISRTime = 0;
unsigned long lastPressTime = 0;
unsigned long pressStartTime = 0;
bool waitingForSecondPress = false;

const unsigned long debounceDelay = 50;
const unsigned long longPressTime = 3000;
const unsigned long doublePressTime = 500;

int yOffset;
int xMid;
int yMid;
char pout[50];

float theta = 0.0;
float psi = 0.0;
float phi = 0.0;
int xScale = 0;
int yScale = 0;

enum PressType {
  NoPress,
  SinglePress,
  DoublePress,
  LongPress
};

enum MachineState {
  Off,
  TwoAxis,
  Xaxis,
  Yaxis,
  RawData,
  Length
};

volatile MachineState currentState = Off;
volatile PressType currentPress = NoPress;

void drawTriangle(int x, int y, int size, const String &direction){
  for (int i = 0; i < size; i++){
    for (int j = 0; j <= i; j++){
      if (direction == "up"){
        myOLED.pixel(x + j, y + i);
        myOLED.pixel(x - j, y + i);
      }
      else if (direction == "down"){
        myOLED.pixel(x + j, y - i);
        myOLED.pixel(x - j, y - i);
      }
      else if (direction == "right"){
        myOLED.pixel(x - i, y + j);
        myOLED.pixel(x - i, y - j);
      }
      else if (direction == "left"){
        myOLED.pixel(x + i, y + j);
        myOLED.pixel(x + i, y - j);
      }
    }
  }
}

float getXAngle(){
  float x = imu.data.accelX;
  float y = imu.data.accelY;
  float z = imu.data.accelZ;
  return atan2(x, sqrt(y*y + z*z)) * 180.0 / M_PI;
}

float getYAngle(){
  float x = imu.data.accelX;
  float y = imu.data.accelY;
  float z = imu.data.accelZ;
  return atan2(y, sqrt(x*x + z*z)) * 180.0 / M_PI;
}

void buttonPressISR() {
  unsigned long now = millis();
  if (now - lastISRTime > debounceDelay) {
    buttonTriggered = true;
    lastISRTime = now;
  }
}

void handleButtonPress() {
  unsigned long now = millis();

  if (buttonTriggered) {
    buttonTriggered = false;
    pressStartTime = now;

    // Detect long press
    while (digitalRead(switchPin) == HIGH) {
      if (millis() - pressStartTime >= longPressTime) {
        currentPress = LongPress;
        waitingForSecondPress = false;
        return;
      }
    }

    // Handle double press
    if (waitingForSecondPress && (now - lastPressTime < doublePressTime)) {
      currentPress = DoublePress;
      waitingForSecondPress = false;
    } else {
      waitingForSecondPress = true;
      lastPressTime = now;
    }
  }

  // Handle single press timeout
  if (waitingForSecondPress && (now - lastPressTime > doublePressTime)) {
    currentPress = SinglePress;
    waitingForSecondPress = false;
  }
}

void setup() {
  Wire.begin();
  
  delay(1000);
  Serial.begin(9600);
  delay(1000);
  pinMode(switchPin, INPUT_PULLDOWN);
  attachInterrupt(switchPin, buttonPressISR, FALLING);

  while (!myOLED.begin()) {
    Serial.println("- Device Begin Failed");
  }

  while(imu.beginI2C(i2cAddress) != BMI2_OK){
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
  }

  // Size: 48y , 64x
  yOffset = (myOLED.getHeight() - myOLED.getFont()->height) / 2;
  xMid = myOLED.getWidth() / 2;
  yMid = myOLED.getHeight() / 2;

  for (int i = 0; i < myOLED.getWidth(); i++) {
    for (int j = 0; j < myOLED.getHeight(); j++) {
      myOLED.pixel(i, j);
    }
  }
  myOLED.display();
  myOLED.erase();
}

void loop() {
  handleButtonPress();

  if (currentPress == DoublePress){
    currentState = (MachineState)(((int)currentState + 1) % (int)Length);
    currentState = (MachineState)max((int)currentState, 1);
  }
  if (currentPress == LongPress){
    currentState = Off;
  }
  if (currentPress != NoPress) {
    Serial.print("Current State: ");
    Serial.println((int)currentState);
    currentPress = NoPress; // Reset press type
  }
  
  imu.getSensorData();
  theta = getXAngle();
  psi = getYAngle();
  xScale = int(map(theta, -90, 90, -12, 12));
  yScale = int(map(psi, -90, 90, -12, 12));
  // Serial.print("Theta: ");
  // Serial.print(theta);
  // Serial.print("  Psi: ");
  // Serial.println(psi);
  myOLED.erase();
  switch(currentState){
    case Off:
      
      break;
    case TwoAxis:
      if (xScale > 0){
        drawTriangle(xMid + 22, yMid, abs(xScale), "right");
      }else{
        drawTriangle(xMid - 22, yMid, abs(xScale), "left");
      }
      if (yScale > 0){
        drawTriangle(xMid, yMid - 22, abs(yScale), "up");
      }else{
        drawTriangle(xMid, yMid + 22, abs(yScale), "down");
      }

      break;
    case Xaxis:
      if (xScale > 0){
        drawTriangle(xMid + 22, yMid, abs(xScale), "right");
      }else{
        drawTriangle(xMid - 22, yMid, abs(xScale), "left");
      }

      break;
    case Yaxis:
      if (yScale > 0){
        drawTriangle(xMid, yMid - 22, abs(yScale), "up");
      }else{
        drawTriangle(xMid, yMid + 22, abs(yScale), "down");
      }

      break;
    case RawData:
      sprintf(pout, "AX:%1.3f", imu.data.accelX);
      myOLED.text(0, yOffset-16, pout);
      sprintf(pout, "AY:%1.3f", imu.data.accelY);
      myOLED.text(0, yOffset, pout);
      sprintf(pout, "AZ:%1.3f", imu.data.accelZ);
      myOLED.text(0, yOffset+16, pout);
      
      break;
  }
  myOLED.display();
  
  delay(250);
}