/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 八路灰度传感器引脚定义 (接在GPIOA的PA0~PA7)
#define SENSOR_NUM 8
#define SENSOR_GPIO GPIOA
#define SENSOR_PIN_0 GPIO_PIN_0  // LINE_SENSOR_1
#define SENSOR_PIN_1 GPIO_PIN_1  // LINE_SENSOR_2
#define SENSOR_PIN_2 GPIO_PIN_2  // LINE_SENSOR_3
#define SENSOR_PIN_3 GPIO_PIN_3  // LINE_SENSOR_4
#define SENSOR_PIN_4 GPIO_PIN_4  // LINE_SENSOR_5
#define SENSOR_PIN_5 GPIO_PIN_5  // LINE_SENSOR_6
#define SENSOR_PIN_6 GPIO_PIN_6  // LINE_SENSOR_7
#define SENSOR_PIN_7 GPIO_PIN_7  // LINE_SENSOR_8

// 电机控制引脚定义
#define MOTOR_STBY_GPIO      GPIOB
#define MOTOR_STBY_PIN       GPIO_PIN_12

#define MOTOR_AIN1_GPIO      GPIOB
#define MOTOR_AIN1_PIN       GPIO_PIN_0
#define MOTOR_AIN2_GPIO      GPIOB
#define MOTOR_AIN2_PIN       GPIO_PIN_1

#define MOTOR_BIN1_GPIO      GPIOB
#define MOTOR_BIN1_PIN       GPIO_PIN_10
#define MOTOR_BIN2_GPIO      GPIOB
#define MOTOR_BIN2_PIN       GPIO_PIN_11

// PWM通道 (TIM4 CH1-CH2)
#define MOTOR_A_PWM_CHANNEL  TIM_CHANNEL_1  // PB6 - MOTOR_PWMA
#define MOTOR_B_PWM_CHANNEL  TIM_CHANNEL_2  // PB7 - MOTOR_PWMB

// 编码器引脚定义
// 左编码器 - TIM2 (PA15=CH1, PB3=CH2)
#define ENCODER_LEFT_TIM     htim2
// 右编码器 - TIM3 (PA6=CH1, PA7=CH2) - 注意：如果PA6/PA7被传感器占用，需要调整
#define ENCODER_RIGHT_TIM    htim3

// 编码器参数
#define ENCODER_RESOLUTION   11        // 编码器线数（根据实际编码器调整）
#define GEAR_RATIO           30        // 减速比（根据实际电机调整）
#define ENCODER_TOTAL_COUNT  (ENCODER_RESOLUTION * GEAR_RATIO * 4)  // 一圈总计数（4倍频）
#define WHEEL_DIAMETER       65.0f     // 轮子直径(mm)
#define WHEEL_PERIMETER      (WHEEL_DIAMETER * 3.14159f)  // 轮子周长(mm)

// 速度参数（高速循迹优化）
#define SPEED_MAX 999         
#define SPEED_MIN 200
#define SPEED_BASE 850        // 基础速度（85%）
#define SPEED_STRAIGHT 950    // 直线最高速
#define SPEED_SLOW 600        // 急弯减速
#define SPEED_TURN_MIN 400    // 转弯最小速度

// PID参数
#define KP 150                // 比例系数
#define KD 80                 // 微分系数

// 板载LED（假设接在PC13，STM32F103C8常用引脚）
#define LED_GPIO GPIOC
#define LED_PIN  GPIO_PIN_13

// 测试按键（可选，如果没有可以用复位按钮）
#define KEY_GPIO GPIOB
#define KEY_PIN  GPIO_PIN_15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t sensor_data[SENSOR_NUM]; // 存储8路传感器数据
const uint16_t sensor_pins[SENSOR_NUM] = {
    SENSOR_PIN_0, SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3,
    SENSOR_PIN_4, SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7
};

// 循迹算法变量
int16_t position = 0;           // 当前位置偏差 (-70到70)
int16_t last_position = 0;      // 上次位置偏差
int16_t position_derivative = 0; // 位置微分
int16_t power_diff = 0;         // 左右轮速度差

// 位置权重（中间传感器权重大）
const int8_t position_weights[SENSOR_NUM] = {-70, -50, -30, -10, 10, 30, 50, 70};

// 测试模式变量
uint8_t test_mode = 0;          // 测试模式: 0=循迹, 1=电机测试
uint32_t led_blink_interval = 1000; // LED闪烁间隔

// 编码器变量
int32_t encoder_left_count = 0;    // 左编码器计数
int32_t encoder_right_count = 0;   // 右编码器计数
int32_t encoder_left_last = 0;     // 上次左编码器计数
int32_t encoder_right_last = 0;    // 上次右编码器计数
float speed_left = 0.0f;           // 左轮速度 (mm/s)
float speed_right = 0.0f;          // 右轮速度 (mm/s)
float distance_left = 0.0f;        // 左轮行驶距离 (mm)
float distance_right = 0.0f;       // 右轮行驶距离 (mm)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ReadSensors(void);
void Encoder_Init(void);
void Encoder_Read(void);
void Encoder_Reset(void);
void Encoder_Calculate_Speed(float delta_time);
void Motor_Init(void);
void Motor_SetSpeed(uint16_t motor_a_speed, uint16_t motor_b_speed);
void Motor_Forward(uint16_t motor_a_speed, uint16_t motor_b_speed);
void Motor_Backward(uint16_t motor_a_speed, uint16_t motor_b_speed);
void Motor_TurnLeft(uint16_t speed);
void Motor_TurnRight(uint16_t speed);
void Motor_Stop(void);
void Motor_DifferentialDrive(int16_t left_speed, int16_t right_speed);
int16_t Calculate_Position(void);
void TrackLine_Advanced(void);
void TrackLine(void);
void Test_Motor_Simple(void);
void LED_Toggle(void);
void LED_Blink_Fast(void);
void LED_Blink_Slow(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  读取8路灰度传感器数据
  * @note   赛道特征：中间白色，两边黑色边框
  *         传感器硬件输出：浅色(白) -> 0(GPIO_PIN_RESET)，深色(黑) -> 1(GPIO_PIN_SET)
  *         映射规则：白色赛道 -> 1，黑色边框 -> 0
  * @retval None
  */
void ReadSensors(void)
{
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        // HAL_GPIO_ReadPin 返回 GPIO_PIN_SET (1) 或 GPIO_PIN_RESET (0)
        // 硬件：白色 -> 0(GPIO_PIN_RESET)，黑色 -> 1(GPIO_PIN_SET)
        // 逻辑映射：白色赛道 -> 1，黑色边框 -> 0
        GPIO_PinState raw = HAL_GPIO_ReadPin(SENSOR_GPIO, sensor_pins[i]);
        sensor_data[i] = (raw == GPIO_PIN_SET) ? 0 : 1;  // 黑色->0, 白色->1
    }
}

/**
  * @brief  编码器初始化
  * @note   启动TIM2和TIM3的编码器模式
  *         需要在CubeMX中配置TIM2和TIM3为Encoder Mode
  * @retval None
  */
void Encoder_Init(void)
{
    // 启动编码器模式
    HAL_TIM_Encoder_Start(&ENCODER_LEFT_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER_RIGHT_TIM, TIM_CHANNEL_ALL);
    
    // 复位计数器
    __HAL_TIM_SET_COUNTER(&ENCODER_LEFT_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODER_RIGHT_TIM, 0);
}

/**
  * @brief  读取编码器计数值
  * @retval None
  */
void Encoder_Read(void)
{
    // 读取编码器计数（16位定时器需要处理溢出）
    encoder_left_count = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    encoder_right_count = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
    
    // 计算行驶距离（累积）
    distance_left += (encoder_left_count - encoder_left_last) * WHEEL_PERIMETER / ENCODER_TOTAL_COUNT;
    distance_right += (encoder_right_count - encoder_right_last) * WHEEL_PERIMETER / ENCODER_TOTAL_COUNT;
}

/**
  * @brief  复位编码器
  * @retval None
  */
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_LEFT_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODER_RIGHT_TIM, 0);
    
    encoder_left_count = 0;
    encoder_right_count = 0;
    encoder_left_last = 0;
    encoder_right_last = 0;
    distance_left = 0.0f;
    distance_right = 0.0f;
    speed_left = 0.0f;
    speed_right = 0.0f;
}

/**
  * @brief  计算编码器速度
  * @param  delta_time: 时间间隔 (秒)
  * @retval None
  */
void Encoder_Calculate_Speed(float delta_time)
{
    // 读取当前编码器值
    int32_t left_current = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    int32_t right_current = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
    
    // 计算增量
    int32_t left_delta = left_current - encoder_left_last;
    int32_t right_delta = right_current - encoder_right_last;
    
    // 计算速度 (mm/s)
    if (delta_time > 0.0f)
    {
        speed_left = (left_delta * WHEEL_PERIMETER / ENCODER_TOTAL_COUNT) / delta_time;
        speed_right = (right_delta * WHEEL_PERIMETER / ENCODER_TOTAL_COUNT) / delta_time;
    }
    
    // 更新上次值
    encoder_left_last = left_current;
    encoder_right_last = right_current;
    encoder_left_count = left_current;
    encoder_right_count = right_current;
}

/**
  * @brief  电机初始化
  * @retval None
  */
void Motor_Init(void)
{
    // 启动PWM
    HAL_TIM_PWM_Start(&htim4, MOTOR_A_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, MOTOR_B_PWM_CHANNEL);
    
    // 使能电机驱动 (STBY引脚置高)
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO, MOTOR_STBY_PIN, GPIO_PIN_SET);
    
    // 初始停止状态
    Motor_Stop();
}

/**
  * @brief  设置电机速度
  * @param  motor_a_speed: 电机A速度 (0-1000)
  * @param  motor_b_speed: 电机B速度 (0-1000)
  * @retval None
  */
void Motor_SetSpeed(uint16_t motor_a_speed, uint16_t motor_b_speed)
{
    // 限制速度范围
    if (motor_a_speed > SPEED_MAX) motor_a_speed = SPEED_MAX;
    if (motor_b_speed > SPEED_MAX) motor_b_speed = SPEED_MAX;
    
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_A_PWM_CHANNEL, motor_a_speed);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_B_PWM_CHANNEL, motor_b_speed);
}

/**
  * @brief  电机前进
  * @param  motor_a_speed: 电机A速度
  * @param  motor_b_speed: 电机B速度
  * @retval None
  */
void Motor_Forward(uint16_t motor_a_speed, uint16_t motor_b_speed)
{
    // 电机A正转
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    
    // 电机B正转
    HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    
    Motor_SetSpeed(motor_a_speed, motor_b_speed);
}

/**
  * @brief  电机后退
  * @param  motor_a_speed: 电机A速度
  * @param  motor_b_speed: 电机B速度
  * @retval None
  */
void Motor_Backward(uint16_t motor_a_speed, uint16_t motor_b_speed)
{
    // 电机A反转
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    
    // 电机B反转
    HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_SET);
    
    Motor_SetSpeed(motor_a_speed, motor_b_speed);
}

/**
  * @brief  原地左转
  * @param  speed: 转向速度
  * @retval None
  */
void Motor_TurnLeft(uint16_t speed)
{
    // 电机A反转，电机B正转
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    
    Motor_SetSpeed(speed, speed);
}

/**
  * @brief  原地右转
  * @param  speed: 转向速度
  * @retval None
  */
void Motor_TurnRight(uint16_t speed)
{
    // 电机A正转，电机B反转
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    
    HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_SET);
    
    Motor_SetSpeed(speed, speed);
}

/**
  * @brief  电机停止
  * @retval None
  */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    
    Motor_SetSpeed(0, 0);
}

/**
  * @brief  差速驱动（支持负速度）
  * @param  left_speed: 左轮速度（可为负）
  * @param  right_speed: 右轮速度（可为负）
  * @retval None
  */
void Motor_DifferentialDrive(int16_t left_speed, int16_t right_speed)
{
    // 左电机方向控制
    if (left_speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, GPIO_PIN_SET);
        left_speed = -left_speed;
    }
    
    // 右电机方向控制
    if (right_speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, GPIO_PIN_SET);
        right_speed = -right_speed;
    }
    
    // 限幅
    if (left_speed > SPEED_MAX) left_speed = SPEED_MAX;
    if (right_speed > SPEED_MAX) right_speed = SPEED_MAX;
    
    Motor_SetSpeed(left_speed, right_speed);
}

/**
  * @brief  计算赛道中心位置（加权平均）
  * @note   白色赛道=1，黑色边框=0
  *         通过检测白色区域来判断小车在赛道的位置
  * @retval 位置值 (-70到70，0表示中心）
  */
int16_t Calculate_Position(void)
{
    int32_t weighted_sum = 0;
    uint8_t sensor_count = 0;
    
    // 加权求和 - 检测白色赛道
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        if (sensor_data[i] == 1) // 检测到白色赛道
        {
            weighted_sum += position_weights[i];
            sensor_count++;
        }
    }
    
    // 计算位置
    if (sensor_count > 0)
    {
        return (int16_t)(weighted_sum / sensor_count);
    }
    else
    {
        // 全黑（完全偏离赛道）：保持上次位置进行修正
        return last_position;
    }
}

/**
  * @brief  高级循迹算法 - PID控制 + 自适应速度
  * @retval None
  */
void TrackLine_Advanced(void)
{
    // 计算当前位置
    position = Calculate_Position();
    
    // 计算微分
    position_derivative = position - last_position;
    last_position = position;
    
    // PID计算速度差
    power_diff = (KP * position / 10) + (KD * position_derivative);
    
    // 根据偏差调整基础速度（直线加速，弯道减速）
    int16_t base_speed;
    int16_t abs_pos = (position < 0) ? -position : position;
    
    if (abs_pos < 15) // 基本在直线上
    {
        base_speed = SPEED_STRAIGHT;
    }
    else if (abs_pos < 40) // 小弯
    {
        base_speed = SPEED_BASE;
    }
    else // 急弯
    {
        base_speed = SPEED_SLOW;
    }
    
    // 计算左右轮速度
    int16_t left_speed = base_speed - power_diff;
    int16_t right_speed = base_speed + power_diff;
    
    // 速度限幅
    if (left_speed < SPEED_TURN_MIN) left_speed = SPEED_TURN_MIN;
    if (right_speed < SPEED_TURN_MIN) right_speed = SPEED_TURN_MIN;
    if (left_speed > SPEED_MAX) left_speed = SPEED_MAX;
    if (right_speed > SPEED_MAX) right_speed = SPEED_MAX;
    
    // 差速控制
    Motor_DifferentialDrive(left_speed, right_speed);
}

/**
  * @brief  循迹算法 - 根据传感器数据控制电机
  * @note   传感器排列: S0 S1 S2 S3 S4 S5 S6 S7 (左->右)
  *         白色赛道为1，黑色边框为0
  *         检测白色区域来保持在赛道中央
  * @retval None
  */
void TrackLine(void)
{
    // 简单算法作为备用
    uint8_t left = sensor_data[0] + sensor_data[1] + sensor_data[2];
    uint8_t right = sensor_data[5] + sensor_data[6] + sensor_data[7];
    
    if (sensor_data[3] == 1 && sensor_data[4] == 1) // 中心有白色赛道
    {
        Motor_Forward(SPEED_STRAIGHT, SPEED_STRAIGHT);
    }
    else if (left == 0) // 左侧全黑（左边框），需要右转
    {
        Motor_Forward(SPEED_BASE, SPEED_TURN_MIN);
    }
    else if (right == 0) // 右侧全黑（右边框），需要左转
    {
        Motor_Forward(SPEED_TURN_MIN, SPEED_BASE);
    }
    else if (sensor_data[2] == 1 || sensor_data[3] == 1) // 白色偏左，需要左转
    {
        Motor_Forward(SPEED_SLOW, SPEED_BASE);
    }
    else if (sensor_data[4] == 1 || sensor_data[5] == 1) // 白色偏右，需要右转
    {
        Motor_Forward(SPEED_BASE, SPEED_SLOW);
    }
    else // 异常情况
    {
        Motor_Forward(SPEED_BASE, SPEED_BASE);
    }
}

/**
  * @brief  LED翻转
  * @retval None
  */
void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO, LED_PIN);
}

/**
  * @brief  LED快速闪烁（指示传感器检测到黑线）
  * @retval None
  */
void LED_Blink_Fast(void)
{
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time > 100) // 100ms闪烁
    {
        last_time = HAL_GetTick();
        LED_Toggle();
    }
}

/**
  * @brief  LED慢速闪烁（指示正常运行）
  * @retval None
  */
void LED_Blink_Slow(void)
{
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time > 500) // 500ms闪烁
    {
        last_time = HAL_GetTick();
        LED_Toggle();
    }
}

/**
  * @brief  简化的电机测试（最大功率持续运行）
  * @note   LED快闪表示正在测试
  * @retval None
  */
void Test_Motor_Simple(void)
{
    static uint32_t last_time = 0;
    static uint8_t started = 0;
    
    // LED快速闪烁表示测试模式
    LED_Blink_Fast();
    
    // 启动后持续以最大功率前进
    if (!started)
    {
        started = 1;
        last_time = HAL_GetTick();
        Motor_Forward(SPEED_MAX, SPEED_MAX); // 最大功率前进
    }
    
    // 每10秒输出一次运行时间（通过LED闪烁次数计数）
    if (HAL_GetTick() - last_time > 10000)
    {
        last_time = HAL_GetTick();
        // 让LED暂停一下表示已运行10秒
        HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 初始化电机
  Motor_Init();
  
  // 初始化编码器
  Encoder_Init();
  
  // 调试：确认TIM4配置
  // ARR值应该是9999或999
  uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim4);
  
  // 通过LED闪烁次数指示ARR范围（调试用）
  if (arr_value < 1000) {
      // ARR是999，快闪1次
      for(int i=0; i<2; i++) { HAL_GPIO_TogglePin(LED_GPIO, LED_PIN); HAL_Delay(100); }
  } else if (arr_value < 10000) {
      // ARR是9999，快闪2次  
      for(int i=0; i<4; i++) { HAL_GPIO_TogglePin(LED_GPIO, LED_PIN); HAL_Delay(100); }
  } else {
      // ARR更大，快闪3次
      for(int i=0; i<6; i++) { HAL_GPIO_TogglePin(LED_GPIO, LED_PIN); HAL_Delay(100); }
  }
  
  // LED指示系统启动（快闪3次）
  for(int i = 0; i < 6; i++)
  {
      HAL_GPIO_TogglePin(LED_GPIO, LED_PIN);
      HAL_Delay(200);
  }
  
  HAL_Delay(500); // 暂停后进入下一个指示
  
  // 默认进入循迹模式
  // 如果需要测试电机，将test_mode改为1
  test_mode = 0; // 0=循迹模式, 1=电机测试

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t last_speed_calc = HAL_GetTick(); // 用于速度计算的时间戳
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 每100ms计算一次速度
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_calc >= 100)
    {
        float delta_time = (current_time - last_speed_calc) / 1000.0f;
        Encoder_Calculate_Speed(delta_time);
        last_speed_calc = current_time;
    }
    
    if (test_mode == 0) // 循迹模式
    {
        ReadSensors();
        TrackLine_Advanced(); // 使用高级PID循迹
        LED_Blink_Slow(); // 慢速闪烁表示循迹模式
        HAL_Delay(5); // 更快的循环频率
    }
    else // 电机测试模式
    {
        Test_Motor_Simple();
        HAL_Delay(50);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
