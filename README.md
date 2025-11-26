# STM32 智能小车循迹项目

基于 STM32F103C8 的八路灰度传感器循迹智能小车项目。

## 📦 硬件清单

- STM32F103C8T6 最小系统板 × 1
- 八路灰度传感器模块 × 1
- **MG310 直流减速电机（带霍尔/GMR编码器）× 2**
  - 减速比：1:20
  - 额定电压：7.4V（建议使用7-13V）
  - 额定转速：400±13% RPM
  - 编码器：13线霍尔或GMR编码器
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

### 4. 编码器模块连接（可选）

| 编码器 | A相引脚 | B相引脚 | 说明 |
|-------|---------|---------|------|
| 左编码器 | PA15 (TIM2_CH1) | PB3 (TIM2_CH2) | 测量左轮速度 |
| 右编码器 | PB4 (TIM3_CH1) | PB5 (TIM3_CH2) | 测量右轮速度 |
| VCC | 5V | 编码器供电 |
| GND | GND | 接地 |

**注意**：
- 如果PA6/PA7被灰度传感器占用，右编码器可改用PB4/PB5
- MG310电机内置13线霍尔/GMR编码器，输出AB两相信号
- 编码器参数已在代码中配置为：13线×20减速比×4倍频=1040计数/圈

### 5. MG310电机性能参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 型号 | MG310 | 霍尔/GMR编码器版本 |
| 减速比 | 1:20 | 已在代码中配置 |
| 额定电压 | 7.4V | 建议使用7-13V |
| 额定转速 | 400±13% RPM | 输出轴转速 |
| 空载转速 | 500±13% RPM | 无负载时 |
| 空载电流 | ≤220mA | 7.4V时 |
| 额定电流 | ≤500mA | 7.4V负载时 |
| 堵转电流 | ≤2.0A | 最大电流 |
| 额定扭矩 | 0.4 kgf·cm | 持续输出 |
| 堵转扭矩 | 1.5± kgf·cm | 瞬时最大 |
| 编码器 | 13线霍尔/GMR | 一圈1040计数(4倍频) |
| 接口 | PH2.0 6P | 电源+编码器 |
| 重量 | 70g | 单个电机 |

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

### 第六步：配置编码器定时器（可选）

#### 6.1 配置 TIM2（左编码器）

1. 点击 **Timers → TIM2**
2. 设置：
   - **Combined Channels**: Encoder Mode
   - **Encoder Mode**: Encoder Mode TI1 and TI2
   - **Parameter Settings**:
     - **Counter Period (ARR)**: 65535 (16位定时器最大值)
     - **Prescaler**: 0

#### 6.2 配置 TIM3（右编码器）

1. 点击 **Timers → TIM3**
2. 设置：
   - **Combined Channels**: Encoder Mode
   - **Encoder Mode**: Encoder Mode TI1 and TI2
   - **Parameter Settings**:
     - **Counter Period (ARR)**: 65535
     - **Prescaler**: 0

#### 6.3 确认引脚分配

- TIM2_CH1: **PA15** (左编码器A相)
- TIM2_CH2: **PB3** (左编码器B相)
- TIM3_CH1: **PB4** (右编码器A相)
- TIM3_CH2: **PB5** (右编码器B相)

### 第七步：配置时钟树 (Clock Configuration)

1. 点击顶部 **Clock Configuration** 标签页
2. 设置：
   - **Input frequency (HSE)**: 8 MHz
   - **PLLMUL**: x9 (使系统时钟达到 72MHz)
   - **System Clock Mux**: PLLCLK
   - **APB1 Prescaler**: /2 (APB1 = 36MHz)
   - **APB2 Prescaler**: /1 (APB2 = 72MHz)
3. 确认 **HCLK** 显示为 72 MHz

### 第七步：配置时钟树 (Clock Configuration)

1. 点击顶部 **Clock Configuration** 标签页
2. 设置：
   - **Input frequency (HSE)**: 8 MHz
   - **PLLMUL**: x9 (使系统时钟达到 72MHz)
   - **System Clock Mux**: PLLCLK
   - **APB1 Prescaler**: /2 (APB1 = 36MHz)
   - **APB2 Prescaler**: /1 (APB2 = 72MHz)
3. 确认 **HCLK** 显示为 72 MHz

### 第八步：配置 USART1 (可选，用于调试)

1. 点击 **Connectivity → USART1**
2. 设置：
   - **Mode**: Asynchronous
   - **Baud Rate**: 115200 Bits/s
   - **Word Length**: 8 Bits
   - **Parity**: None
   - **Stop Bits**: 1

### 第九步：项目设置与代码生成

#### 9.1 项目管理设置

1. 点击顶部 **Project Manager** 标签页
2. **Project** 子标签：
   - **Project Name**: Smare_Car
   - **Project Location**: 选择您的项目路径
   - **Toolchain/IDE**: MDK-ARM V5

3. **Code Generator** 子标签：
   - 勾选 `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`
   - 勾选 `Keep User Code when re-generating`
   - 勾选 `Delete previously generated files when not re-generated`

#### 9.2 生成代码

1. 点击右上角 **GENERATE CODE** 按钮
2. 等待代码生成完成
3. 点击 **Open Project** 打开 Keil 项目

### 第十步：在 Keil 中编译

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
// 速度参数 (0-999)
#define SPEED_BASE 850        // 基础速度（85%）
#define SPEED_STRAIGHT 950    // 直线最高速
#define SPEED_SLOW 600        // 急弯减速
#define SPEED_TURN_MIN 400    // 转弯最小速度

// PID参数（用于高级循迹算法）
#define KP 120                // 比例系数（已优化）
#define KD 120                // 微分系数（已优化）

// 编码器参数（MG310电机）
#define ENCODER_RESOLUTION   13        // 霍尔/GMR编码器线数
#define GEAR_RATIO           20        // 减速比 1:20
#define MOTOR_RATED_RPM      400       // 额定转速 400±13% RPM
#define WHEEL_DIAMETER       48.0f     // 轮子直径(mm)，根据实际测量

// 主循环延时 (毫秒)
HAL_Delay(5);  // 高速循迹建议5ms
```

## 🎛️ 编码器使用说明（可选功能）

### 编码器变量

项目已集成编码器支持，可用于：
- 实时测量左右轮速度（mm/s）
- 计算累计行驶距离（mm）
- 实现速度闭环控制

### 可用函数

```c
Encoder_Init();                          // 初始化编码器
Encoder_Read();                          // 读取编码器计数
Encoder_Calculate_Speed(0.1f);           // 计算速度（参数为时间间隔）
Encoder_Reset();                         // 复位编码器

// 全局变量
extern float speed_left;                 // 左轮速度 (mm/s)
extern float speed_right;                // 右轮速度 (mm/s)
extern float distance_left;              // 左轮行驶距离 (mm)
extern float distance_right;             // 右轮行驶距离 (mm)
```

### 编码器参数校准

**MG310电机参数（已配置）：**
- ✅ 编码器线数：13线（霍尔/GMR编码器）
- ✅ 减速比：1:20
- ✅ 额定转速：400±13% RPM
- ✅ 一圈总计数：1040 (13×20×4倍频)
- ⚠️ 轮子直径：48mm（**需根据实际测量调整**）

**校准步骤：**
1. 用卡尺测量轮子外径（单位mm）
2. 修改 `main.c` 中 `WHEEL_DIAMETER` 参数
3. 测试：让小车前进一段距离，对比实际距离和编码器读数
4. 微调直径参数直到误差<5%

## 🔧 常见问题排查

### 1. 电机不转

- 检查 L298N 供电是否正常（**MG310需要7-13V，推荐7.4V/12V锂电池**）
- 检查 STM32 与 L298N 是否共地
- 检查电机驱动引脚连接是否正确
- 使用万用表测量 PWMA/PWMB 是否有 PWM 输出
- **MG310空载电流<220mA，堵转电流<2A，确保驱动能力足够**

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

### 4. 速度测量与编码器

- **已集成**：项目已包含编码器驱动代码
- 通过光电编码器测量实际速度，实现速度闭环控制
- 可用于精确定位、里程计算等高级功能
- 如需使用需在CubeMX中配置TIM2和TIM3为编码器模式

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
