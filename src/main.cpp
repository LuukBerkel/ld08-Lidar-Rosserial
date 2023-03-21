#include <Arduino.h>
#include <ld08.hpp>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#define RXD2 16
#define PWM  15

ld08 lidar = ld08(RXD2, PWM);
ld08_frame frame;

ros::NodeHandle node_handler;
sensor_msgs::LaserScan lidar_msg;
ros::Publisher laser_pub("laser_scan_publisher", &lidar_msg);

void setup() {
  lidar.begin();
  node_handler.initNode();
  node_handler.advertise(laser_pub);

  // Mallocing buffers that are used for reading
  lidar_msg.ranges = (float*)malloc(sizeof(float)*POINT_PER_PACK);
  lidar_msg.ranges_length = POINT_PER_PACK;
  lidar_msg.intensities = (float*)malloc(sizeof(float)*POINT_PER_PACK);
  lidar_msg.intensities_length = POINT_PER_PACK;
}

void loop() {
  if (lidar.read_frame(&frame)){
         lidar_msg.header.frame_id = "base_scan";
      uint32_t diff =
          ((uint32_t)frame.end_angle + 36000 - (uint32_t)frame.start_angle) % 36000;
        lidar_msg.angle_increment = diff / (POINT_PER_PACK - 1) / 100.0;
        lidar_msg.angle_min = static_cast<double>(frame.start_angle) / 100.0;
        lidar_msg.angle_max = static_cast<double>(frame.end_angle % 36000) / 100.0;
        lidar_msg.time_increment = (360 / frame.speed) / (( lidar_msg.angle_max -   lidar_msg.angle_min) /  lidar_msg.angle_increment);
        lidar_msg.range_max = 0; 
        lidar_msg.range_min = 0;
        lidar_msg.scan_time = 360 / frame.speed;
        for (size_t i = 0; i < POINT_PER_PACK; i++)
        {
           if (lidar_msg.range_max < (frame.point[i].distance / 1000.f)){
              lidar_msg.range_max = frame.point[i].distance / 1000.f;
              lidar_msg.intensities[0] = frame.point[i].confidence;
           }
           else if (lidar_msg.range_min > (frame.point[i].distance / 1000.f)){
              lidar_msg.angle_min = frame.point[i].distance / 1000.f;
           }
          lidar_msg.ranges[i] = frame.point[i].distance / 1000.f;
          lidar_msg.intensities[i] = frame.point[i].confidence;
        }
        laser_pub.publish(&lidar_msg);
        node_handler.spinOnce();
  } 
}
