# STM32 智能小车循迹项目

基于 STM32F103C8 的八路灰度传感器循迹智能小车项目。

## 📦 硬件清单

- STM32F103C8T6 最小系统板 × 1
- 八路灰度传感器模块 × 1
- 直流电机 × 2
- L298N 电机驱动模块 × 1
- 小车底盘 × 1
- 电池组（7.4V 或 12V）× 1
- 杜邦线若干

## 🔌 硬件连线

### 1. 八路灰度传感器连接

| 传感器引脚 | STM32 引脚 | 说明 |
|-----------|-----------|------|
| S0 | PA0 | 最左侧传感器 (LINE_SENSOR_1) |
| S1 | PA1 | 左侧传感器 (LINE_SENSOR_2) |
| S2 | PA2 | 左中传感器 (LINE_SENSOR_3) |
| S3 | PA3 | 左中心传感器 (LINE_SENSOR_4) |
| S4 | PA4 | 右中心传感器 (LINE_SENSOR_5) |
| S5 | PA5 | 右中传感器 (LINE_SENSOR_6) |
| S6 | PA6 | 右侧传感器 (LINE_SENSOR_7) |
| S7 | PA7 | 最右侧传感器 (LINE_SENSOR_8) |
| VCC | 5V | 传感器供电 |
| GND | GND | 接地 |

**注意**：传感器排列顺序从左到右为 S0 → S7

### 2. 电机驱动模块连接

| 驱动模块引脚 | STM32 引脚 | 功能说明 |
|------------|-----------|---------|
| STBY | PB12 | 待机控制 (高电平使能) |
| AIN1 | PB0 | 电机A方向控制1 |
| AIN2 | PB1 | 电机A方向控制2 |
| BIN1 | PB10 | 电机B方向控制1 |
| BIN2 | PB11 | 电机B方向控制2 |
| PWMA | PB6 (TIM4_CH1) | 电机A PWM 调速 |
| PWMB | PB7 (TIM4_CH2) | 电机B PWM 调速 |
| VM | 电池正极 (7.4V-12V) | 电机供电 |
| VCC | 5V | 逻辑电路供电 |
| GND | 电池负极 + STM32 GND | 共地 |

### 3. 电源连接

- **方案1**：L298N 的 5V 输出 → STM32 的 5V 引脚
- **方案2**：独立 5V 电源给 STM32 供电
- **重要**：务必将 L298N 的 GND 与 STM32 的 GND 连接（共地）

## ⚙️ STM32CubeMX 完整配置流程

### 第一步：新建项目

1. 打开 **STM32CubeMX**
2. 点击 **File → New Project**
3. 在芯片选择界面搜索 `STM32F103C8`
4. 选择 **STM32F103C8Tx** (LQFP48 封装)
5. 点击 **Start Project**

### 第二步：配置系统时钟 (RCC)

1. 在左侧 **Pinout & Configuration** 标签页
2. 点击 **System Core → RCC**
3. 设置：
   - **High Speed Clock (HSE)**: Crystal/Ceramic Resonator
   - **Low Speed Clock (LSE)**: Disable

### 第三步：配置调试接口 (SYS)

1. 点击 **System Core → SYS**
2. 设置：
   - **Debug**: Serial Wire (用于 ST-Link 调试)

### 第四步：配置 GPIO

#### 4.1 灰度传感器输入引脚 (PA0~PA7)

1. 在芯片引脚图上点击 **PA0**，选择 `GPIO_Input`
2. 重复操作，将 **PA1, PA2, PA3, PA4, PA5, PA6, PA7** 都设置为 `GPIO_Input`
3. 点击 **System Core → GPIO**，配置 PA0~PA7：
   - **GPIO mode**: Input mode
   - **GPIO Pull-up/Pull-down**: Pull-up (上拉，根据传感器特性调整)
   - **User Label**: 可选，如 `SENSOR_0` ~ `SENSOR_7`

#### 4.2 电机方向控制输出引脚 (PB0, PB1, PB10, PB11, PB12)

1. 在芯片引脚图上点击 **PB0**，选择 `GPIO_Output`
2. 重复操作，将 **PB1, PB10, PB11, PB12** 都设置为 `GPIO_Output`
3. 点击 **System Core → GPIO**，配置这些引脚：
   - **GPIO mode**: Output Push Pull
   - **GPIO Pull-up/Pull-down**: No pull-up and no pull-down
   - **Maximum output speed**: High
   - **User Label**: 可选，如 `MOTOR_AIN1`, `MOTOR_AIN2`, `MOTOR_BIN1`, `MOTOR_BIN2`, `MOTOR_STBY`

### 第五步：配置 TIM4 PWM

#### 5.1 启用 TIM4 通道

1. 点击 **Timers → TIM4**
2. 设置：
   - **Clock Source**: Internal Clock
   - **Channel1**: PWM Generation CH1
   - **Channel2**: PWM Generation CH2

#### 5.2 配置 PWM 参数

1. 在 **Parameter Settings** 标签页：
   - **Prescaler (PSC)**: 71 (假设系统时钟 72MHz，产生 1MHz 计数频率)
   - **Counter Mode**: Up
   - **Counter Period (AutoReload Register - ARR)**: 999 (产生 1kHz PWM 频率)
   - **Internal Clock Division**: No Division
   - **auto-reload preload**: Enable

2. 在 **PWM Generation Channel 1** 和 **Channel 2** 下：
   - **Mode**: PWM mode 1
   - **Pulse (CCR)**: 0 (初始占空比为 0)
   - **Output compare preload**: Enable
   - **Fast Mode**: Disable
   - **CH Polarity**: High

#### 5.3 确认引脚分配

- TIM4_CH1 应自动分配到 **PB6** (MOTOR_PWMA)
- TIM4_CH2 应自动分配到 **PB7** (MOTOR_PWMB)

**如果引脚冲突**：
- 右键点击冲突引脚，选择 `Reset_State`
- 再次点击 PB6，选择 `TIM4_CH1`
- 点击 PB7，选择 `TIM4_CH2`

### 第六步：配置时钟树 (Clock Configuration)

1. 点击顶部 **Clock Configuration** 标签页
2. 设置：
   - **Input frequency (HSE)**: 8 MHz
   - **PLLMUL**: x9 (使系统时钟达到 72MHz)
   - **System Clock Mux**: PLLCLK
   - **APB1 Prescaler**: /2 (APB1 = 36MHz)
   - **APB2 Prescaler**: /1 (APB2 = 72MHz)
3. 确认 **HCLK** 显示为 72 MHz

### 第七步：配置 USART1 (可选，用于调试)

1. 点击 **Connectivity → USART1**
2. 设置：
   - **Mode**: Asynchronous
   - **Baud Rate**: 115200 Bits/s
   - **Word Length**: 8 Bits
   - **Parity**: None
   - **Stop Bits**: 1

### 第八步：项目设置与代码生成

#### 8.1 项目管理设置

1. 点击顶部 **Project Manager** 标签页
2. **Project** 子标签：
   - **Project Name**: Smare_Car
   - **Project Location**: 选择您的项目路径
   - **Toolchain/IDE**: MDK-ARM V5

3. **Code Generator** 子标签：
   - 勾选 `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`
   - 勾选 `Keep User Code when re-generating`
   - 勾选 `Delete previously generated files when not re-generated`

#### 8.2 生成代码

1. 点击右上角 **GENERATE CODE** 按钮
2. 等待代码生成完成
3. 点击 **Open Project** 打开 Keil 项目

### 第九步：在 Keil 中编译

1. 在 Keil MDK 中打开生成的项目
2. 找到 `main.c` 文件（项目已包含循迹代码）
3. 点击 **Build** (F7) 编译项目
4. 编译成功后，连接 ST-Link 下载到 STM32

## 🎯 传感器校准

### 传感器极性说明

- **黑线（轨道线）**：传感器输出低电平 (0)
- **白线（背景）**：传感器输出高电平 (1)

**如果您的传感器极性相反**，需要修改 `main.c` 中 `TrackLine()` 函数的逻辑，将所有 `== 0` 改为 `== 1`，反之亦然。

### 测试方法

1. 将传感器模块放在白纸上，观察 LED 指示灯状态
2. 将传感器放在黑线上，观察 LED 状态变化
3. 通过调节传感器上的电位器，调整灵敏度

## 📝 代码参数调整

在 `main.c` 中可以调整以下参数：

```c
// 速度参数 (0-1000)
#define SPEED_NORMAL 600    // 直行速度
#define SPEED_TURN 300      // 转弯时内侧轮速度
#define SPEED_MAX 1000      // 最大速度

// 主循环延时 (毫秒)
HAL_Delay(10);  // 建议范围：5-20ms
```

## 🔧 常见问题排查

### 1. 电机不转

- 检查 L298N 供电是否正常（电池电压）
- 检查 STM32 与 L298N 是否共地
- 检查电机驱动引脚连接是否正确
- 使用万用表测量 ENA/ENB 是否有 PWM 输出

### 2. 电机转向错误

- 交换对应电机的两根线
- 或修改代码中 IN1/IN2（或 IN3/IN4）的电平状态

### 3. 传感器读数异常

- 检查传感器供电是否为 5V
- 检查传感器与 STM32 的信号线连接
- 调节传感器灵敏度电位器
- 确认传感器极性是否与代码匹配

### 4. 小车不循迹

- 先测试传感器是否能正确识别黑白线
- 调整速度参数（降低 SPEED_NORMAL）
- 调整循环延时（减小 HAL_Delay 值提高响应速度）
- 检查轨道宽度与传感器间距是否匹配

### 5. 编译错误

- 确保 Keil 使用 ARM Compiler 6
- 检查项目文件中 `ToolsetNumber` 为 `0x6`
- 确认所有外设初始化函数已在 CubeMX 中正确生成

## 📊 循迹算法说明

项目采用基于位置的循迹策略：

| 传感器状态 | 动作 | 说明 |
|-----------|------|------|
| S3 和 S4 检测到黑线 | 直行 | 小车在轨道中心 |
| S0 或 S1 检测到黑线 | 大幅左转 | 右轮快，左轮慢 |
| S2 检测到黑线 | 小幅左转 | 轻微调整方向 |
| S6 或 S7 检测到黑线 | 大幅右转 | 左轮快，右轮慢 |
| S5 检测到黑线 | 小幅右转 | 轻微调整方向 |
| 全部为白线 | 慢速前进 | 寻找轨道 |

## 🚀 进阶功能

### 1. PID 循迹算法

当前使用简单的阈值判断，可升级为 PID 控制实现更平滑的循迹。

### 2. 蓝牙控制

通过 USART 连接蓝牙模块，实现远程启动/停止控制。

### 3. OLED 显示

添加 OLED 屏幕显示传感器状态和速度信息。

### 4. 速度测量

通过光电编码器测量实际速度，实现闭环控制。

## 📚 参考资料

- [STM32F103C8 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [STM32 HAL 库开发手册](https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [L298N 电机驱动模块说明](https://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf)

## 📄 许可证

本项目采用 MIT 许可证。

## 👨‍💻 作者

STM32 智能小车项目

---

**祝您调试顺利！如有问题欢迎交流。** 🚗
