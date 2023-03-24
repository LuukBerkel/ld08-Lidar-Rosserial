// #include <Arduino.h>
// #include <ld08.hpp>
// #include <ros.h>
// #include <std_msgs/String.h>
// #include <sensor_msgs/LaserScan.h>

// ros::NodeHandle node_handler;

// sensor_msgs::LaserScan lidar_msg;
// ros::Publisher laser_pub("laser_scan_publisher", &lidar_msg);

// void setup() {
//   // Starting rosserial.
//   node_handler.initNode();
//   node_handler.advertise(laser_pub);

//   // Starting lidar.
//   lidar.begin();

//   //  Buffers that are used for reading.
// }

// void loop() {
//   if (lidar.read_frame(&frame)){
//     lidar_msg.header.frame_id = "base_scan";
//     uint32_t diff = ((uint32_t)frame.end_angle + 36000 - (uint32_t)frame.start_angle) % 36000;
//     lidar_msg.angle_increment = diff / (POINT_PER_PACK - 1) / 100.0;
//     lidar_msg.angle_min = static_cast<double>(frame.start_angle) / 100.0;
//     lidar_msg.angle_max = static_cast<double>(frame.end_angle % 36000) / 100.0;
//     lidar_msg.time_increment = (360 / frame.speed) / (( lidar_msg.angle_max -   lidar_msg.angle_min) /  lidar_msg.angle_increment);
//     lidar_msg.range_max = 0; 
//     lidar_msg.range_min = 0;
//     lidar_msg.scan_time = 360 / frame.speed;
//     for (size_t i = 0; i < POINT_PER_PACK; i++)
//     {
//       if (lidar_msg.range_max < (frame.point[i].distance / 1000.f)){
//         lidar_msg.range_max = frame.point[i].distance / 1000.f;
//         lidar_msg.intensities[0] = frame.point[i].confidence;
//       }
//       else if (lidar_msg.range_min > (frame.point[i].distance / 1000.f)){
//         lidar_msg.angle_min = frame.point[i].distance / 1000.f;
//       }
//       lidar_msg.ranges[i] = frame.point[i].distance / 1000.f;
//       lidar_msg.intensities[i] = frame.point[i].confidence;
      
//       laser_pub.publish(&lidar_msg);
//       node_handler.spinOnce();
//     }
//   }
//   delay(1000);
// }

#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Arduino.h>
#include <ld08.hpp>
#include <queue>
#include <math.h>
#include <algorithm>

#define ANGLE_TO_RADIAN(angle) ((angle) * 3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle) * 180000 / 3141.59)

#define RXD2 16
#define PWM  15

// Rosserial settings
ros::NodeHandle nh;
sensor_msgs::LaserScan output;
ros::Publisher chatter("laser_scan_publisher", &output);

// Lidar settings
ld08 lidar = ld08(RXD2, PWM);
ld08_frame frame;
TaskHandle_t rosserial_handle;
 
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  output.ranges = (float*)malloc(sizeof(float)*POINT_PER_PACK);
  output.ranges_length = POINT_PER_PACK;
  output.intensities = (float*)malloc(sizeof(float)*POINT_PER_PACK);
  output.intensities_length = POINT_PER_PACK;

  lidar.begin();
}

void loop()
{
    if(lidar.read_frame(&frame)){
      int angle_min = ANGLE_TO_RADIAN(frame.start_angle);
      int angle_max = ANGLE_TO_RADIAN(frame.end_angle);
      float range_min = 0.0;
      float range_max = 100.0;
      int angle_increment = ANGLE_TO_RADIAN(frame.speed / 2300);

      // Adjust the parameters according to the demand
      output.header.frame_id = "base_scan";
      output.angle_min = angle_min;
      output.angle_max = angle_max;
      output.range_max = range_max;
      output.range_min = range_min;
      output.angle_increment = angle_increment;
      //output.time_increment =  (360.0f / frame.speed) / ((frame.speed - angle_min) / angle_increment);
      output.scan_time = 360.0f / frame.speed;
    
      // Pushing pack
      for (size_t i = 0; i < POINT_PER_PACK; i++)
      {
        output.ranges[i] = frame.point[i].distance / 1000.f;
        output.intensities[i] = frame.point[i].confidence;
      }

      // Publishing message.
      chatter.publish( &output);
    }

    nh.spinOnce();
}
  

