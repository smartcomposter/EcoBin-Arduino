#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 28
#define ADAFRUITBLE_RDY 3
#define ADAFRUITBLE_RST 26

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

//Thresholds
const int inside_temp_max = 55;
const int outside_temp_max = 80;
const int inside_temp_min = 65;
const unsigned long motor_on_duration = 60000; //1 minute
const unsigned long motor_off_duration = 60000*60*4; //4 hours

const int motor_pin = 48;
const int inside_temp_sensor_pin = A15;
const int outside_temp_sensor_pin = A14;
const int moisture_sensor_pin = A7;
const int heating_pin = 34;
const int methane_sensor_pin = 29;

const int numReadings = 50;

//Temp variables
const float temp_initial_value = 25;
const float R1 = 10000, c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float inside_temp_values[numReadings];
float inside_temp_total = temp_initial_value * numReadings;
int inside_temp_current_index = 0;
float outside_temp_values[numReadings];
float outside_temp_total = temp_initial_value * numReadings;
int outside_temp_current_index = 0;

//Moisture variables
const float moisture_initial_value = 0;
float moisture_values[numReadings];
float moisture_total = moisture_initial_value * numReadings;
int moisture_current_index = 0;

//Methane variables
const float methane_initial_value = 0;
const float m = -0.318; //Slope
const float b = 1.133; //Y-Intercept
float methane_values[numReadings];
float methane_total = methane_initial_value * numReadings;
int methane_current_index = 0;

//Heating variables
bool heating_on = false;

//Motor variables
bool motor_on = false;
unsigned long motor_off_start_time = 0;
unsigned long motor_on_start_time = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(motor_pin, OUTPUT);
  pinMode(heating_pin, OUTPUT);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    inside_temp_values[thisReading] = temp_initial_value;
    outside_temp_values[thisReading] = temp_initial_value;
    moisture_values[thisReading] = moisture_initial_value;
    methane_values[thisReading] = methane_initial_value;
  }
  
  BTLEserial.setDeviceName("BLETemp");
  BTLEserial.begin();
}

void loop() {
  BTLEserial.pollACI();
  aci_evt_opcode_t status = BTLEserial.getState();

  float inside_temp = averageTemperature(inside_temp_sensor_pin, &inside_temp_total, inside_temp_values, &inside_temp_current_index);
  float outside_temp = averageTemperature(outside_temp_sensor_pin, &outside_temp_total, outside_temp_values, &outside_temp_current_index);
  float moisture_value = averageMoisture();
  float methane_value = averageMethane();

  if (status == ACI_EVT_CONNECTED) {
    if (BTLEserial.available()) {
      while (BTLEserial.available()) {
        char c = BTLEserial.read();
        if (c ==  '1') {
          motor_on = !motor_on;
          if (motor_on == false) {
            digitalWrite(motor_pin, LOW); 
          } else {
            digitalWrite(motor_pin, HIGH);
          }
        }
        Serial.print(c);
      }
      Serial.println(); 
    }

    String inside_temp_string = "Temp: " + String(inside_temp);
    String outside_temp_string = "Temp2: " + String(outside_temp);
    String moisture_string = "Moisture: " + String(moisture_value);
    String methane_string = "Methane: " + String(methane_value);
    
    uint8_t sendbuffer[30];
    
    inside_temp_string.getBytes(sendbuffer, 30);
    char sendbuffersize = min(30, inside_temp_string.length());
    Serial.print("Inside Temperature: "); Serial.print((char *)sendbuffer); Serial.println("'C");
    BTLEserial.write(sendbuffer, sendbuffersize);

    outside_temp_string.getBytes(sendbuffer, 30);
    sendbuffersize = min(30, outside_temp_string.length());
    Serial.print("Outside Temperature: "); Serial.print((char *)sendbuffer); Serial.println("'C");
    BTLEserial.write(sendbuffer, sendbuffersize);

    moisture_string.getBytes(sendbuffer, 30);
    sendbuffersize = min(30, moisture_string.length());
    Serial.print("Moisture: "); Serial.print((char *)sendbuffer); Serial.println("%");
    BTLEserial.write(sendbuffer, sendbuffersize);

    methane_string.getBytes(sendbuffer, 30);
    sendbuffersize = min(30, methane_string.length());
    Serial.print("Methane: "); Serial.print((char *)sendbuffer); Serial.println("%");
    BTLEserial.write(sendbuffer, sendbuffersize);

    Serial.println();
    
  } else if (status == ACI_EVT_DISCONNECTED) {
    Serial.println("Not Connected");
  }

  //heating logic
  if ((inside_temp > inside_temp_max || outside_temp > outside_temp_max) && heating_on == true || motor_on == true) {
    digitalWrite(heating_pin, LOW);
    Serial.println("*******HEATING OFF********");
    heating_on = false;
  } else if ((inside_temp < inside_temp_min) && heating_on == false) {
    digitalWrite(heating_pin, HIGH);
    Serial.println("****************************HEATING ON************************************");
    heating_on = true;
  }

  //motor logic
  if (motor_on) {
    unsigned long elapsed_time_on = millis() - motor_on_start_time;
    if (elapsed_time_on > motor_on_duration) {
      digitalWrite(motor_pin, LOW);
      motor_off_start_time = millis();
      motor_on = false;
    }
  } else {
    unsigned long elapsed_time_off = millis() - motor_off_start_time;
    if (elapsed_time_off > motor_off_duration || motor_off_start_time == 0) {
      digitalWrite(motor_pin, HIGH);
      motor_on_start_time = millis();
      motor_on = true;
    }
  }
}

//http://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
float averageTemperature(int temp_sensor_pin, float *temp_total, float *temp_values, int *current_index) {
  *temp_total = *temp_total - temp_values[*current_index];

  float Vo, logR2, R2, T, Tc, Tf;
  
  Vo = analogRead(temp_sensor_pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;

  return averageValue(Tc, temp_total, temp_values, current_index);
}

//https://learn.sparkfun.com/tutorials/soil-moisture-sensor-hookup-guide#Calibration
float averageMoisture() {
  moisture_total = moisture_total - moisture_values[moisture_current_index];  
  float moustire_sensor_output_value = analogRead(moisture_sensor_pin);
  float mapped_value = map(moustire_sensor_output_value, 0, 885, 0, 100);

  return averageValue(mapped_value, &moisture_total, moisture_values, &moisture_current_index);
}

//https://www.geekstips.com/mq4-sensor-natural-gas-methane-arduino/
float averageMethane() {
  methane_total = methane_total - methane_values[methane_current_index];  

  //Calculate R0
  float sensor_volt, RS_air, R0, sensorValue;
  for (int x = 0 ; x < 10 ; x++) {
    sensorValue = sensorValue + analogRead(methane_sensor_pin);
  }
  sensorValue = sensorValue / 10.0;
  sensor_volt = sensorValue * (5.0 / 1023.0);
  RS_air = ((5.0 * 10.0) / sensor_volt) - 10.0;
  R0 = RS_air / 4.4;

  //Calculate Gas Concentration
  float RS_gas, ratio;
  sensorValue = analogRead(methane_sensor_pin);
  sensor_volt = sensorValue * (5.0 / 1023.0);
  RS_gas = ((5.0 * 10.0) / sensor_volt) - 10.0;
  ratio = RS_gas / R0;

  double ppm_log = (log10(ratio) - b) / m;
  double ppm = pow(10, ppm_log);
  double methane_percentage = ppm / 10000;

  return averageValue(methane_percentage, &methane_total, methane_values, &methane_current_index);
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
