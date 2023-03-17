#include <Arduino.h>
#include <ld08.hpp>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#define RXD2 16
#define PWM  15

ld08 lidar = ld08(RXD2, PWM);
ld08_frame frame;

ros::NodeHandle nh;
sensor_msgs::LaserScan lidar_msg;
ros::Publisher laser_pub("lidar", &lidar_msg);

void messageCb(const std_msgs::String&) {
}

ros::Subscriber<std_msgs::String> sub("your_topic", &messageCb);


    



void setup() {
  lidar.begin();
  nh.initNode();
  nh.advertise(laser_pub);
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

  nh.spinOnce();
}