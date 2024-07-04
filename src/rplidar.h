#pragma once 

#ifndef ESP32
  #error This example runs on ESP32
#endif

#include <LDS_RPLIDAR_A1.h>
#define LED_BUILTIN 2
const uint8_t LDS_MOTOR_EN_PIN = 19; // ESP32 Dev Kit LiDAR enable pin
const uint8_t LDS_MOTOR_PWM_PIN = 27; // LiDAR motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ    10000
#define LDS_MOTOR_PWM_BITS    11
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LiDAR motor speed control
// #define LidarSerial Serial2

static HardwareSerial LidarSerial(2); // TX 17, RX 16
static LDS_RPLIDAR_A1 lidar;  // Uncomment your LiDAR model
static u_int16_t measurement_buffer[360];
static int measurement_size = 0;
static bool measurement_ready = false; 
static float hz = 10.0;



class rplidarHandler 
{ 
public:
    static void init();
    static void loop();
    static size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length);
    static void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed);
    static void lidar_info_callback(LDS::info_t code, String info);
    static void lidar_error_callback(LDS::result_t code, String aux_info);
    static void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin);
    static void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed);

    static bool is_measurement_ready();
    static u_int16_t get_measurement(int angle_deg);
    static int lidar_serial_read_callback();

    static void reset_measurement();
    static float get_fq();








};
