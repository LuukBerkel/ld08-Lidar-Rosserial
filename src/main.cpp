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
  

