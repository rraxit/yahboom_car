#include "rplidar.h"


void rplidarHandler::init()
{

  Serial.print("LiDAR model ");
  Serial.println(lidar.getModelName());

  Serial.print("LiDAR RX buffer size "); // default 128 hw + 256 sw
  Serial.print(LidarSerial.setRxBufferSize(1024)); // must be before .begin()

  uint32_t baud_rate = lidar.getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);
  LidarSerial.begin(baud_rate); // Use default GPIO TX 17, RX 16

  while (LidarSerial.read() >= 0)
    delay(1000);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.init();

  reset_measurement();

  LDS::result_t result = lidar.start();
  Serial.print("LiDAR start() result: ");
  Serial.println(lidar.resultCodeToString(result));

  if (result < 0)
    Serial.println("WARNING: is LDS connected to ESP32?");
}



size_t rplidarHandler::lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void rplidarHandler::lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  static int i=0;

  if ((i++ % 20 == 0) || scan_completed) {
    // Serial.print(i);
    // Serial.print(' ');
    // Serial.print(distance_mm);
    // Serial.print(' ');
    // Serial.print(angle_deg);
    int index = (int) angle_deg;
    u_int16_t value = (u_int16_t) distance_mm;

    measurement_buffer[index] = value;
  

    if (scan_completed)
    {
    //   Serial.println('*');
      measurement_size = i;
      i = 0;
      measurement_ready = true; 
      hz = lidar.getCurrentScanFreqHz();
    }
    // else
    //   Serial.println();
  }
}

void rplidarHandler::lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LDS info ");
  Serial.print(lidar.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void rplidarHandler::lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LDS error ");
  Serial.print(lidar.resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void rplidarHandler::lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ?
    LDS_MOTOR_EN_PIN : LDS_MOTOR_PWM_PIN;

  if (value <= LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void rplidarHandler::lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  // Uncomment for debug
 /*
  Serial.print("Packet ");
  Serial.print(length);
  Serial.print("b");
  float hz = lidar.getCurrentScanFreqHz();
  Serial.print(", speed ");
  Serial.println(hz);
 
 */


}

float rplidarHandler::get_fq()
{
  return hz; 
}


bool rplidarHandler::is_measurement_ready()
{
    return measurement_ready;
}

u_int16_t rplidarHandler::get_measurement(int angle_deg)
{
    if(angle_deg >=360)
      return 0;    
    measurement_ready = false;
    return  measurement_buffer[angle_deg];
}



void rplidarHandler::loop() {
  lidar.loop();
}


int rplidarHandler::lidar_serial_read_callback() {
  int c = LidarSerial.read();
// Uncomment below for debug
/*
  if (c < 0)
    return c;

  if (c < 16)
    Serial.print('0');
  Serial.print(c, HEX);

  static int i=0;
  if (i++ % 16 == 0)
    Serial.println();
  else
    Serial.print(' ');
*/
  return c;
}

void rplidarHandler::reset_measurement()
{
  for(int i = 0; i < 360; ++i)
    measurement_buffer[i] = 0;
}