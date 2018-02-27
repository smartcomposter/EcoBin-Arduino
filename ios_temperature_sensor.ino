#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

const int numReadings = 50;
const int initialValue = 25;

const float R1 = 10000;
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int Vo;
float logR2, R2, T, Tc, Tf;

int readings[numReadings];
int readIndex = 0;
int total = initialValue * numReadings;

const int sensorPin = A0;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = initialValue;
  }
  
  BTLEserial.setDeviceName("BLETemp");
  BTLEserial.begin();
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  BTLEserial.pollACI();

  aci_evt_opcode_t status = BTLEserial.getState();
  if (status != laststatus) {
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    String temperatureString = averageTemperature();
    
    uint8_t sendbuffer[30];
    String finalString = "Temp: " + temperatureString;
    finalString.getBytes(sendbuffer, 30);
    char sendbuffersize = min(30, finalString.length());

    Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

//   if (BTLEserial.available()) {
//      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
//    }
//    // OK while we still have something to read, get a character and print it out
//    while (BTLEserial.available()) {
//      char c = BTLEserial.read();
//      Serial.print(c);
//    }

    BTLEserial.write(sendbuffer, sendbuffersize);
    
//    Serial.println(BTLEserial.read());
  } else if (status == ACI_EVT_DISCONNECTED) {
//    Serial.println("NOT CONNECTED");
  }


//  Serial.println(BTLEserial.read());
}

String averageTemperature() {
  int average = averageValue(sensorPin);
  return String(average);
}

int averageValue(int inputPin) {
  total = total - readings[readIndex];
  
  Vo = analogRead(inputPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  
  readings[readIndex] = Tc;
  Serial.print("Current Temp: "); Serial.println(readings[readIndex]);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  
  return total / numReadings;
}
//
//String temperature(int sensorVal) {
////  float voltage = (sensorVal / 1024.0) * 5.0;
////  float temperature = (voltage - .5) * 100;
//  Serial.println(sensorVal);
//  return String(sensorVal);
//}
