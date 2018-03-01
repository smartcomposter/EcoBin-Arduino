#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

const int numReadings = 50;

const int temp_sensor_pin = A0;
const int moisture_sensor_pin = A2;


const float temp_initial_value = 25;
const float R1 = 10000;
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float Vo, logR2, R2, T, Tc, Tf;
float temp_values[numReadings];
float temp_total = temp_initial_value * numReadings;
int temp_current_index = 0;

const float moisture_initial_value = 0;
float moustire_sensor_output_value;
float moisture_values[numReadings];
float moisture_total = moisture_initial_value * numReadings;
int moisture_current_index = 0;


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    temp_values[thisReading] = temp_initial_value;
    moisture_values[thisReading] = moisture_initial_value;
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
    uint8_t sendbuffer[30];
    String temperatureString = averageTemperature();
//    String moistureString = averageMoisture();
    
    temperatureString.getBytes(sendbuffer, 30);
    char sendbuffersize = min(30, temperatureString.length());
    Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");
    BTLEserial.write(sendbuffer, sendbuffersize);
//
//    moistureString.getBytes(sendbuffer, 30);
//    sendbuffersize = min(30, moistureString.length());
//    Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");
//    BTLEserial.write(sendbuffer, sendbuffersize);
    
  } else if (status == ACI_EVT_DISCONNECTED) {
    Serial.println("Not Connected");
  }
}

String averageTemperature() {
  temp_total = temp_total - temp_values[temp_current_index];
  
  Vo = analogRead(temp_sensor_pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  
//  temp_values[temp_current_index] = Tc;
//  temp_total = temp_total + temp_values[temp_current_index];
//  temp_current_index = temp_current_index + 1;
//
//  if (temp_current_index >= numReadings) {
//    temp_current_index = 0;
//  }
//  
//  float average = temp_total / numReadings;
  float average = averageValue(Tc, &temp_total, temp_values, &temp_current_index);
  
  return "Temp: " + String(average);
}

String averageMoisture() {
  moisture_total = moisture_total - moisture_values[moisture_current_index];
  
  moustire_sensor_output_value = analogRead(moisture_sensor_pin);
  
//  moisture_values[moisture_current_index] = moustire_sensor_output_value;
//  moisture_total = moisture_total + moisture_values[moisture_current_index];
//  moisture_current_index = moisture_current_index + 1;
//
//  if (moisture_current_index >= numReadings) {
//    moisture_current_index = 0;
//  }
//  
//  float average = moisture_total / numReadings;
  float average = averageValue(moustire_sensor_output_value, &moisture_total, moisture_values, &moisture_current_index);

  return "Moisture: " + String(average);
}

float averageValue(float current_value, float *total_value, float *all_values, int *current_index) {
  all_values[*current_index] = current_value;
  *total_value = *total_value + all_values[*current_index];
  *current_index = *current_index + 1;

  if (*current_index >= numReadings) {
    *current_index = 0;
  }
  
  float average = *total_value / numReadings;
  return average;
}
