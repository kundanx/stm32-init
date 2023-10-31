#include "robot.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"


#include <cmath>
#include "robotlib/maths/math.hpp"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

// extern DMA_HandleTypeDef hdma_usart3_rx;
// extern DMA_HandleTypeDef hdma_usart2_rx;
// extern DMA_HandleTypeDef hdma_uart4_rx;

#define BASE_DIAMETER (0.73)
#define WHEEL_DIAMETER (0.13)
#define MAX_VELOCITY (2.0) 
#define MAX_OMEGA (2.0)

Robot::Robot(UART_HandleTypeDef *random_huart):
   omniwheel_kinematics(BASE_DIAMETER,WHEEL_DIAMETER)
{
  rhuart = random_huart;
}

void Robot::init()
{
  base_motor[0]= Motor(&htim10,GPIOE, TIM_CHANNEL_1, GPIO_PIN_0);
  base_motor[1]= Motor(&htim9, GPIOE, TIM_CHANNEL_1, GPIO_PIN_1);
  base_motor[2]= Motor(&htim9, GPIOE, TIM_CHANNEL_2, GPIO_PIN_4);
  base_motor[3]= Motor(&htim14, GPIOC, TIM_CHANNEL_1, GPIO_PIN_13);

  //joystick.init();

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  if(HAL_UART_Receive_DMA(rhuart, this->recieved_data,recieve_buffer_size) == HAL_OK)
  {
    printf("dma initiallized \n");
  }
  else 
    printf("dma not initiallized \n");

}

void Robot::run()
{   
  printf("%u \n", control_data.jdata.button1);
  set_state_from_joystick_data(&control_data);
  omniwheel_kinematics.get_motor_omega(robot_state.base_twist, motor_omegas);
  get_pwm(motor_omegas, motor_pwm);

  //motor 0
  TIM10->CCR1 = motor_pwm[0];
  if(motor_direction[0] == clockwise) 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
  else 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  
  // Motor 1
  TIM9->CCR1 = motor_pwm[1];
  if(motor_direction[1] == clockwise) 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  else 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  // Motor 2
  TIM9->CCR2 = motor_pwm[2];
  if(motor_direction[2] == clockwise)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  // Motor 3
  TIM14->CCR1 = motor_pwm[3];
  if(motor_direction[3] == clockwise)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

 }

void Robot::set_state_from_joystick_data(ControlData *data)
{
  const uint8_t d_band = 100;

  float velocity = 0, heading_angle = robot_state.base_twist.get_theta(), base_omega = 0;
  float speed_factor = 1.0;

  if(abs(control_data.jdata.l_hatx)< d_band && abs(control_data.jdata.l_haty) < d_band 
     && abs(control_data.jdata.lt) < d_band && abs(control_data.jdata.rt) < d_band)
     {
      robot_state.base_twist = Twist(0,0,0);
     }

  if(abs(control_data.jdata.l_hatx) > d_band || abs(control_data.jdata.l_haty) > d_band )
  {

   float  magnitude = sqrt(pow(control_data.jdata.l_hatx, 2) + pow(control_data.jdata.l_haty,2));
    velocity = map<float>(magnitude, 0, 128, 0 , MAX_VELOCITY);
    heading_angle = atan2(control_data.jdata.l_haty, control_data.jdata.l_hatx);
  }

  if(abs(control_data.jdata.lt) > 30)
  {
     base_omega = map<float>(control_data.jdata.lt, 0, 128, 0, MAX_OMEGA);
  }
  else if(abs(control_data.jdata.rt) > 30)
  {
     base_omega = -map<float>(control_data.jdata.lt, 0, 128, 0, MAX_OMEGA);
  }

   robot_state.base_twist = Twist::from_v_theta_omega(velocity, heading_angle, base_omega);

}

void Robot::get_pwm(float *omegas, uint16_t *pwms)
{
  for(int i =0; i<4 ; i++)
  {
    if(omegas[i] > 0)
    {
     pwms[i] = (uint16_t)map((double)omegas[i], 0.0 ,MAX_OMEGA, 0.0, 65525.0);
     motor_direction[i] = clockwise;
    }
    else if(omegas[i] < 0)
    {
     pwms[i] = (uint16_t)(-map((double)omegas[i], 0.0 , -MAX_OMEGA, 0.0, -65525.0));
     motor_direction[i] = anti_clockwise;
    }   
  }
}

Robot robot(&huart4);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    if(huart->Instance == robot.rhuart->Instance)
    {
       HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
       if(robot.recieved_data[0]== 165){

        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
        
          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
         robot.control_data.jdata.button1 = robot.recieved_data[1];
         robot.control_data.jdata.button2 = robot.recieved_data[2];
         robot.control_data.jdata.lt= robot.recieved_data[3];
         robot.control_data.jdata.rt = robot.recieved_data[4];

         robot.control_data.jdata.l_hatx = robot.recieved_data[5];
         robot.control_data.jdata.l_haty = robot.recieved_data[6];
         robot.control_data.jdata.r_hatx = robot.recieved_data[7];
         robot.control_data.jdata.r_haty = robot.recieved_data[8];
        
       }
    }
    HAL_UART_Receive_DMA(robot.rhuart, robot.recieved_data,robot.recieve_buffer_size);
}

uint8_t Robot::generate_checksum(uint8_t *check_variable)
{
   uint8_t value = 0;
   for(int i = 0; i<9; i++)
   {
     value +=check_variable[i];
   }
   return value;
}


void init_robot()
{  
  HAL_Delay(50);
  robot.init();
  printf("Init robot\n"); 

}

void operate_robot()
{
  while (true) {
    robot.run();
    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    HAL_Delay(20);
  }
}



