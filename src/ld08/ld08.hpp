/**
 * @file ld08.hpp
 * @brief this file contains the definitions for the lds_08 driver on an esp32 for arduino.
 * @author Luuk van Berkel
 * @date 16-3-2023
 */
#ifndef LD08
#define LD08

#include <stdint.h> 

/// @brief this struct definition is for the settings of the lds_02.
typedef struct {             
  int speed;
  int angle;
  int pwm_freq;
} ld08_settings;    

/// @brief this struct definition is for the data of the lds_02.
typedef struct {             
  uint16_t distance;
  uint8_t confidence;
} ld08_data;   

/// @brief this struct definition is for the frames of the lds_02.
typedef struct {         
  // represents conditions    
  uint16_t rotation_speed;
  uint16_t start_angle;
  uint16_t end_angle;
  uint16_t timestamp;

  // represents data
  ld08_data data_buffer_ptr[12];
} ld08_frame;     

/// @brief this class definition is for controlling the lidar.
class ld08
{
private:
    // config variables
    int rx_pin;
    int pwm_pin;
    ld08_settings lds_08_setting;

    // internal variables
    uint8_t raw_buffer_ptr[44];
    void (*pwm_callback)(ld08_settings, int);


    /// @brief this function uses the crc table to validate the crc.
    /// @param buffer the buffer that needs to be validated.
    /// @param lenght  the lenght of the buffer.
    /// @param crc the crc that has been send over. 
    /// @return true if valid crc.
    bool validate_crc(uint8_t cmd_byte, uint8_t len_byte, uint8_t* buffer);

    /// @brief this function parses the buffer into a lds_02_frame. 
    bool parse_buffer(ld08_frame* frame, uint8_t* buffer, int lenght);
public:
    // constructors and destructor
    ld08(int rx_pin, int pwm_pin);
    ~ld08();

    /// @brief this function inits the lidar and loads defaults.
    void begin();

    /// @brief read_frame reads the serial buffer to init a frame.
    /// @return frame the pointer to the frame that is going to be used.
    bool read_frame(ld08_frame* frame);

    /// @brief settings changes the lidar settings.
    /// @param speed the rotation speed of the lidar.
    /// @param angle the angle of data gathering.
    /// @param pwm_freq the ration pwm freq.
    void settings(int speed, int angle = 360, int pwm_freq = 10000);

    /// @brief pwm_worker appends a callback on pwm events if set.
    /// @param pwm_callback change in settings the pwm callback is called.
    void pwm_worker( void (*pwm_callback)(ld08_settings, int));
};
#endif