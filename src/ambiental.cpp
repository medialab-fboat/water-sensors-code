// Autoria: Thiago Mattos e Felipe Tinoco
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "mavlink/mavlink.h"
// ** CS - pin 10 
// ** MOSI - pin 11
// ** MOSO - pin 12
// ** SCK - pin 13

SoftwareSerial mySerial(9, 8); // RX, TX
constexpr int chipSelect = 10;
constexpr int voltageDividerRatio = 3;
constexpr int assumedVCC = 5;

int16_t temperatureSensor, pHSensor;
float temperatureSensorVoltage, pHSensorVoltage; 
bool IsMemoryCardInitialized = false;
typedef uint32_t timer;

void ReadSensorData();  
void SendSerialData();
void SendSoftwareSerialData();
void WriteDataToMemoryCard();

void ReadDummySensorData() {
  
  temperatureSensor = random(300,400);
  temperatureSensorVoltage = ((voltageDividerRatio * temperatureSensor) / 1023.f) * assumedVCC; // Probe controller outputs 12V, which passes through a voltage divider to the Arduino analog input. The voltage divider ratio is 3:1.
  pHSensor = random(500,600);
  pHSensorVoltage = ((voltageDividerRatio * pHSensor) / 1023.f) * 5;
}
void SendMAVLinkData();

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("Initializing SD card...");
  /* while (!SD.begin(chipSelect)) {
    Serial.println("Couldn't access the SD card!");
    Serial.println("Check connections and restart Arduino.");
    delay(3000);
  } */
  Serial.println("SDCard initialized successfully!\n");
  randomSeed(millis());
}

void loop() {
  ReadSensorData();
  SendSerialData(); 
  SendMAVLinkData();
  WriteDataToMemoryCard();
}

void ReadSensorData() {
  temperatureSensor = analogRead(A0); 
  temperatureSensorVoltage = ((voltageDividerRatio * temperatureSensor) / 1023.f) * assumedVCC; // Probe controller outputs 12V, which passes through a voltage divider to the Arduino analog input. The voltage divider ratio is 3:1.
  pHSensor = analogRead(A1); 
  pHSensorVoltage = ((voltageDividerRatio * pHSensor) / 1023.f) * 5;
}

void SendMAVLinkData() {
  static timer previous_time = millis();
  if (millis() - previous_time < 1000) return;
  previous_time = millis();

  mavlink_message_t msg;
  uint8_t buffer[64];

  mavlink_msg_param_set_pack(1, 1, &msg, 1, 1, "RUDDER_ANGLE", temperatureSensorVoltage, MAVLINK_TYPE_INT16_T);
  uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
  mySerial.write(buffer, length);
  delay(10);
  
  mavlink_msg_param_set_pack(1, 1, &msg, 1, 1, "SAIL_ANGLE", pHSensorVoltage, MAVLINK_TYPE_INT16_T);
  length = mavlink_msg_to_send_buffer(buffer, &msg);
  mySerial.write(buffer, length); 
  delay(10);
}

void SendSerialData() {
  Serial.print("T");
  Serial.print(millis() / 1000.f);
  Serial.print(";");
  Serial.print("Temp");
  Serial.print(temperatureSensor);
  Serial.print(";");
  Serial.print("pH=");
  Serial.print(pHSensor);
  Serial.print(";");
  Serial.print("TempVoltage"); 
  Serial.print(temperatureSensorVoltage);
  Serial.print(";");
  Serial.print("pHVoltage=");
  Serial.println(pHSensorVoltage);
} 

void SendSoftwareSerialData() {
  mySerial.print(millis() / 1000.f);
  mySerial.print(";");
  mySerial.print(temperatureSensor); 
  mySerial.print(";");
  mySerial.print(pHSensor);
  mySerial.print(";");
  mySerial.print(temperatureSensorVoltage); 
  mySerial.print(";");
  mySerial.println(pHSensorVoltage);
} 

void WriteDataToMemoryCard() {
  File data_file = SD.open("teni6.txt", FILE_WRITE); 
  if(!data_file) {
    Serial.println("Couldn't open teni6.txt");
    return;
  }

  if (!IsMemoryCardInitialized)
  { 
    data_file.print("Time");
    data_file.print(";");
    data_file.print("Temperature");
    data_file.print(";");
    data_file.print("pH");
    data_file.print(";");
    data_file.print("Temperature Voltage");
    data_file.print(";");
    data_file.println("pH Voltage");
    IsMemoryCardInitialized = true;
    data_file.close();
  } else {
    data_file.print(millis() / 1000.f);
    data_file.print(";");
    data_file.print(temperatureSensor);
    data_file.print(";");
    data_file.print(pHSensor);
    data_file.print(";");
    data_file.print(temperatureSensorVoltage);
    data_file.print(";");
    data_file.println(pHSensorVoltage);
    data_file.close();
  }  
}




