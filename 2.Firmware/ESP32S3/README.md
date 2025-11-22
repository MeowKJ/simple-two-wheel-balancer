# ESP32-S3 平衡车项目

基于ESP32-S3的双轮自平衡车，使用MPU6050 IMU、DRV8837电机驱动器和FreeRTOS任务管理。

## 硬件配置

- **主控**: ESP32-S3 (LCKFB-ESP32S3R8N8)
- **IMU**: MPU6050 (I2C接口)
- **电机驱动**: DRV8837 双路H桥
- **编码器**: 7PPR, 减速比1:50
- **其他**: WS2812B LED, 电池电压监测

## 项目结构

```
.
├── platformio.ini              # PlatformIO配置
├── include/
│   └── board.h                 # 硬件引脚定义
├── lib/
│   ├── MotorDriver/            # 电机驱动库 (MCPWM)
│   ├── Encoder/                # 编码器库 (PCNT)
│   ├── IMU/                    # IMU传感器库
│   ├── PIDController/          # PID控制器
│   └── BalancingRobot/         # 平衡车主控制
└── src/
    └── main.cpp                # 主程序 (FreeRTOS任务)
```

## 功能特性

### 1. 模块化设计
- 每个外设封装为独立库
- 清晰的接口定义
- 易于维护和扩展

### 2. RTOS任务管理
- **平衡控制任务** (100Hz, Core 1, 高优先级)
- **遥测任务** (2Hz, Core 0)
- **LED状态指示任务** (10Hz, Core 0)

### 3. 三级PID控制
- **角度PID**: 控制平衡角度
- **速度PID**: 控制前进/后退
- **转向PID**: 控制左右转向

### 4. 硬件外设
- **MCPWM**: 高精度PWM电机控制
- **PCNT**: 硬件正交编码器计数
- **I2C**: MPU6050姿态传感器
- **ADC**: 电池电压监测

## 安装与编译

### 1. 安装依赖

使用PlatformIO安装：

```bash
pio lib install "Adafruit MPU6050"
pio lib install "Adafruit Unified Sensor"
pio lib install "FastLED"
```

### 2. 编译上传

```bash
pio run -t upload
pio device monitor
```

## 使用说明

### 串口命令

- `w` - 前进
- `s` - 后退
- `a` - 左转
- `d` - 右转
- `x` - 停止
- `p` - 调整角度PID的Kp值
- `i` - 调整角度PID的Ki值
- `d` - 调整角度PID的Kd值
- `h` - 显示帮助

### LED指示

- **绿色**: 正常平衡中
- **彩虹渐变**: 未平衡状态

### 遥测信息

串口每500ms输出：
- 俯仰角度
- 左右电机转速 (RPM)
- 电池电压
- 平衡状态

## PID调参指南

### 初始参数

```cpp
角度PID: Kp=40.0, Ki=0.0, Kd=1.5
速度PID: Kp=0.5,  Ki=0.1, Kd=0.0
转向PID: Kp=2.0,  Ki=0.0, Kd=0.1
```

### 调参步骤

1. **调整Kp**: 从小到大增加，使小车能够站立但有振荡
2. **调整Kd**: 增加Kd值减少振荡
3. **调整Ki**: 消除稳态误差（可选）

## 安全提示

- ⚠️ 车体倾斜超过±45°会触发紧急停止
- ⚠️ 首次测试时请注意小车可能突然启动
- ⚠️ 建议在地毯或有摩擦力的地面测试
- ⚠️ 确保电池电压充足 (>6.5V)

## 电路连接

### 左侧电机
- IN1: GPIO45
- IN2: GPIO32
- SLEEP: GPIO46
- ENCODER_A: GPIO37
- ENCODER_B: GPIO36

### 右侧电机
- IN1: GPIO48
- IN2: GPIO0
- SLEEP: GPIO47
- ENCODER_A: GPIO38
- ENCODER_B: GPIO39

### IMU (MPU6050)
- SDA: GPIO4
- SCL: GPIO3
- INT: GPIO7

### 其他
- WS2812B: GPIO14
- Battery ADC: GPIO1

## 故障排除

### 问题1: 小车无法站立
- 检查PID参数是否合适
- 确认IMU方向正确
- 检查电机接线是否正确

### 问题2: 小车剧烈震荡
- 降低Kp值
- 增加Kd值
- 检查编码器是否正常工作

### 问题3: IMU初始化失败
- 检查I2C接线
- 确认MPU6050地址 (0x68 or 0x69)
- 检查电源供应

## 进阶功能

可以扩展的功能：
- 蓝牙/WiFi遥控
- 超声波避障
- 自动巡线
- 手势识别
- APP控制

## 许可证

MIT License

## 作者

基于LCKFB-ESP32S3R8N8开发板