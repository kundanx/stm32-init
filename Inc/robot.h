#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#ifdef __cplusplus

#include "robotlib/actuators/motor.hpp"
#include "robotlib/communication/xbee.hpp"
#include "robotlib/kinematics/omniwheel.hpp"

#include "definations.hpp"



struct ControlData
{
  JoystickData jdata;
  // float rpm[2];
  // float yaw_allign;
};

struct RobotState
{
  Twist base_twist{0, 0, 0};
  //Odometry odom{0, 0, 0};
};

enum direction { clockwise, anti_clockwise};

class Robot {
public:
  Motor base_motor[4];
  // Xbee joystick;
  OmniwheelKinematics omniwheel_kinematics;

  float motor_omegas[4]={0,0,0,0};
  uint16_t motor_pwm[4]={0,0,0,0};
  direction motor_direction[4] = {clockwise, clockwise, clockwise, clockwise};

  RobotState robot_state;
  ControlData control_data;

  uint16_t recieve_buffer_size = 10;
  //uint8_t joystick_data[10] ={0};
  uint8_t recieved_data[10] ={0};


  UART_HandleTypeDef *rhuart;
  void uartCallBack(uint16_t size);


  Robot(UART_HandleTypeDef *random_huart);
  void set_state_from_joystick_data(ControlData *data);
  void get_pwm(float *, uint16_t *);
  uint8_t generate_checksum(uint8_t *check_variable);

  void init();
  void run() ;
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void init_robot();
void operate_robot();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ROBOT_H_
