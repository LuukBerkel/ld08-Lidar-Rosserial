#include <Arduino.h>
#include "ld08/ld08.hpp"

#define RXD2 16
#define PWM  -1

ld08 lidar = ld08(RXD2, PWM);
ld08_frame frame;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting demo of lidar ld08");

  lidar.begin();
  Serial.println("Succesfully started lidar ld08");
}

void loop() { //Choose Serial1 or Serial2 as required
  if (!lidar.read_frame(&frame)){
    Serial.println("Failed to read frame");
  } else {
    Serial.print("Lidar min angle: ");
    Serial.println(frame.start_angle / 100);
    Serial.print("Lidar max angle: ");
    Serial.println(frame.end_angle / 100);
    Serial.print("Rotation speed: ");
    Serial.println(frame.rotation_speed);
    Serial.print("measurments: [");
    for (uint8_t i = 0; i < 12; i++)
    {
      Serial.print("[d: ");
      Serial.print(frame.data_buffer_ptr[i].distance);
      Serial.print("c: ");
      Serial.print(frame.data_buffer_ptr[i].confidence);
      Serial.print("],");
    }
    Serial.println("]");
  }
}