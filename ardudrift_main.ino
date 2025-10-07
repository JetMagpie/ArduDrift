#include "MPU6000.h"
#include "UART.h"
#include "PWMOutput.h"
#include "EEPROM_Manager.h"
#include <math.h>
#include <string.h>

// 参数结构体 - 用于EEPROM存储
struct SystemParams {
    float BOARD_ROTATION;
    float K_GAIN;
    float DEFAULT_GAIN;
    float STEER_BY_ACC_RATE;
    float COUNTER_STEER_RANGE;
    float SERVO_LIMIT_LEFT;
    float SERVO_LIMIT_RIGHT;
    float LOOP_FREQUENCY;
    float IMU_FILTER;
    float SERVO_FILTER;
    float ANGACC_FILTER;
    float STEER_BY_ANGACC_RATE;
};

// 默认参数值
#define DEFAULT_BOARD_ROTATION 90
#define DEFAULT_K_GAIN 0.015
#define DEFAULT_DEFAULT_GAIN 200
#define DEFAULT_STEER_BY_ACC_RATE 2
#define DEFAULT_COUNTER_STEER_RANGE 0.85
#define DEFAULT_SERVO_LIMIT_LEFT 1
#define DEFAULT_SERVO_LIMIT_RIGHT 0.9
#define DEFAULT_LOOP_FREQUENCY 500
#define DEFAULT_IMU_FILTER 20
#define DEFAULT_SERVO_FILTER 20
#define DEFAULT_ANGACC_FILTER 10
#define DEFAULT_STEER_BY_ANGACC_RATE 1

// EEPROM管理器
EEPROM_Manager eeprom(0);
SystemParams current_params;

// 全局变量
volatile bool report_enabled = false;
char serial_buffer[64];
uint8_t serial_index = 0;

// 使用提供的库实例
extern UART uart;
MPU6000 mpu;

// 使用输出1通道 (CH1) - 引脚12
#define SERVO_OUT_CHANNEL PWMOutput::CH1

// 引脚和pwm信号设定
#define STEERING_IN_PIN 2
#define GAIN_IN_PIN 3
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_NEUTRAL 1500
#define INPUT_TIMEOUT_MS 500

// 全局变量
volatile uint32_t steering_start = 0;
volatile uint32_t gain_start = 0;
volatile uint16_t steering_pwm = PWM_NEUTRAL;
volatile uint16_t gain_pwm = PWM_NEUTRAL;
volatile uint32_t steering_last_update = 0;
volatile uint32_t gain_last_update = 0;
float angular_accel_integral = 0.0;

// 参数验证范围结构
struct ParamRange {
    float min;
    float max;
};

// 参数验证范围
const ParamRange param_ranges[] = {
    {0, 360},       // BOARD_ROTATION
    {0.001, 0.1},   // K_GAIN
    {0, 500},      // DEFAULT_GAIN
    {0, 20},      // STEER_BY_ACC_RATE
    {0, 1.0},     // COUNTER_STEER_RANGE
    {0, 1.0},     // SERVO_LIMIT_LEFT
    {0, 1.0},     // SERVO_LIMIT_RIGHT
    {50, 1000},     // LOOP_FREQUENCY
    {1, 500},        // IMU_FILTER
    {1, 500},        // SERVO_FILTER
    {1, 500},        // ANGACC_FILTER
    {0, 10}        // STEER_BY_ANGACC_RATE
};

// 参数名称数组
const char* param_names[] = {
    "BOARD_ROTATION",
    "K_GAIN", 
    "DEFAULT_GAIN",
    "STEER_BY_ACC_RATE",
    "COUNTER_STEER_RANGE",
    "SERVO_LIMIT_LEFT",
    "SERVO_LIMIT_RIGHT", 
    "LOOP_FREQUENCY",
    "IMU_FILTER",
    "SERVO_FILTER",
    "ANGACC_FILTER",
    "STEER_BY_ANGACC_RATE"
};

// 低通滤波器类
class LowPassFilter {
private:
  float alpha;
  float prev_output;
  bool initialized;
  
public:
  LowPassFilter() : initialized(false) {}
  
  void setCutoffFrequency(float cutoff_freq, float sample_freq) {
    float dt = 1.0 / sample_freq;
    float rc = 1.0 / (2.0 * M_PI * cutoff_freq);
    alpha = dt / (dt + rc);
  }
  
  float update(float input) {
    if (!initialized) {
      prev_output = input;
      initialized = true;
    }
    prev_output = prev_output + alpha * (input - prev_output);
    return prev_output;
  }
  
  void reset() {
    initialized = false;
  }
};

// 创建滤波器实例
LowPassFilter accel_x_filter;
LowPassFilter accel_y_filter;
LowPassFilter gyro_z_filter;
LowPassFilter servo_filter;

// 数据结构保存运动学状态
struct KinematicState {
  float accel_x, accel_y, angular_vel;
  float total_accel, accel_direction;
};

// 坐标旋转函数 - 根据飞控板安装方向调整传感器数据
void rotateSensorData(float &accel_x, float &accel_y, float &gyro_z, int rotation_deg) {
  float temp_accel_x, temp_accel_y;
  float temp_gyro_z = gyro_z;
  
  switch(rotation_deg) {
    case 0:   // 标准安装，无需旋转
      // 保持不变
      break;
      
    case 90:  // 顺时针旋转90度
      temp_accel_x = accel_y;    // 原Y轴变为X轴
      temp_accel_y = -accel_x;   // 原负X轴变为Y轴
      // 角速度方向不变（绕Z轴旋转）
      accel_x = temp_accel_x;
      accel_y = temp_accel_y;
      break;
      
    case 180: // 旋转180度
      accel_x = -accel_x;
      accel_y = -accel_y;
      // 角速度方向不变
      break;
      
    case 270: // 顺时针旋转270度（或逆时针90度）
      temp_accel_x = -accel_y;   // 原负Y轴变为X轴
      temp_accel_y = accel_x;    // 原X轴变为Y轴
      // 角速度方向不变
      accel_x = temp_accel_x;
      accel_y = temp_accel_y;
      break;
      
    default:
      // 无效旋转角度，保持不变
      uart.print("警告: 无效的旋转角度: ");
      uart.println((int)rotation_deg);
      break;
  }
}

// PWM输入中断服务函数（保持不变）
void steeringISR() {
  if (digitalRead(STEERING_IN_PIN)) {
    steering_start = micros();
  } else {
    uint32_t pulse_width = micros() - steering_start;
    if (pulse_width >= 800 && pulse_width <= 2200) {
      steering_pwm = pulse_width;
      steering_last_update = millis();
    }
  }
}

void gainISR() {
  if (digitalRead(GAIN_IN_PIN)) {
    gain_start = micros();
  } else {
    uint32_t pulse_width = micros() - gain_start;
    if (pulse_width >= 800 && pulse_width <= 2200) {
      gain_pwm = pulse_width;
      gain_last_update = millis();
    }
  }
}

void checkInputTimeout() {
  uint32_t current_time = millis();
  if (current_time - steering_last_update > INPUT_TIMEOUT_MS) {
    steering_pwm = PWM_NEUTRAL;
  }
  if (current_time - gain_last_update > INPUT_TIMEOUT_MS) {
    gain_pwm = PWM_NEUTRAL + current_params.DEFAULT_GAIN;
  }
}

// 应用传感器滤波器
void applySensorFilters(float accel_x_raw, float accel_y_raw, float gyro_z_raw, 
                       float &accel_x_filt, float &accel_y_filt, float &gyro_z_filt) {
  accel_x_filt = accel_x_filter.update(accel_x_raw);
  accel_y_filt = accel_y_filter.update(accel_y_raw);
  gyro_z_filt = gyro_z_filter.update(gyro_z_raw);
}

// 计算运动学状态 - 添加旋转角度参数
void calculateKinematicState(float accel_x, float accel_y, float angular_vel, 
                            KinematicState &state, int rotation_deg = 0) {
  // 首先根据安装方向旋转传感器数据
  rotateSensorData(accel_x, accel_y, angular_vel, rotation_deg);
  
  // 存储旋转后的数据
  state.accel_x = accel_x;
  state.accel_y = accel_y;
  state.angular_vel = angular_vel;
  
  state.total_accel = sqrt(accel_x * accel_x + accel_y * accel_y);
  state.accel_direction = atan2(accel_y, accel_x) * 180.0 / M_PI;
  
  float angular_vel_rad = fabs(angular_vel) * M_PI / 180.0;
  float total_accel_ms2 = state.total_accel * 9.81;
}

// 简化版反打控制函数 - 维持原有接口
float calculateCounterSteerByKinematics(const KinematicState &state, float gain) {
  // 参数定义
  const float ACCEL_GAIN = current_params.STEER_BY_ACC_RATE;     // 横向加速度增益系数
  const float GYRO_GAIN = 1.0f;     // 角速度增益系数
  const float ANGACC_GAIN = current_params.STEER_BY_ANGACC_RATE;   //角加速度增益系数
  const float DEADBAND_ACCEL = 0.2f; // 加速度死区 (g)
  const float DEADBAND_GYRO = 0.5f;  // 角速度死区 (度/秒)
  
  float counter_steer = 0.0f;
  
  //基于横向加速度的反打分量
  if (fabs(state.accel_y) > DEADBAND_ACCEL) {
    // 加速度与反打方向相反
    float accel_component = -state.accel_y * ACCEL_GAIN;
    counter_steer += accel_component;
  }
  //基于角速度的反打分量
  if (fabs(state.angular_vel) > DEADBAND_GYRO) {
    // 角速度与反打方向相同
    float gyro_component = state.angular_vel * GYRO_GAIN;
    counter_steer += gyro_component;
    //基于角加速度的反打预测
    float angacc_component = current_params.ANGACC_FILTER * (state.angular_vel-angular_accel_integral);
    angular_accel_integral += angacc_component;
    angacc_component *= ANGACC_GAIN/current_params.LOOP_FREQUENCY;
    //counter_steer += angacc_component;
  }
  //应用感度调节
  counter_steer *= gain*current_params.K_GAIN;
  //限制输出范围
  counter_steer = constrain(counter_steer, -current_params.COUNTER_STEER_RANGE, current_params.COUNTER_STEER_RANGE);
  
  return counter_steer;
}

// 计算感度系数
float calculateGain() {
  if (gain_pwm < 1000 || gain_pwm > 2000) {
    return 0.5;
  }
  float normalized = (float)(gain_pwm - 1500) / 500.0;
  normalized = constrain(normalized, 0.0, 1.0);
  return normalized;
}

// 输出到舵机 - 使用CH1通道
void outputServo(float steering_input, float correction) {
  if (steering_input < 1000 || steering_input > 2000) {
    steering_input = PWM_NEUTRAL;
  }
  
  float normalized_input = (steering_input - 1500) / 500.0;
  float final_output = normalized_input + correction;
  final_output = constrain(final_output, -current_params.SERVO_LIMIT_LEFT, current_params.SERVO_LIMIT_RIGHT);
  
  int pwm_output_raw = PWM_NEUTRAL + (int)(final_output * 500);
  pwm_output_raw = constrain(pwm_output_raw, PWM_MIN, PWM_MAX);
  
  float normalized_filtered = servo_filter.update((pwm_output_raw - 1500) / 500.0);
  uint16_t filtered_pwm = PWM_NEUTRAL + (int)(normalized_filtered * 500);
  
  // 使用CH1通道输出
  PWMOutput::setPulse(SERVO_OUT_CHANNEL, filtered_pwm);
}

// 精确循环频率控制
void controlLoopFrequency() {
  static uint32_t last_loop_time = 0;
  uint32_t current_time = micros();
  
  if (last_loop_time > 0) {
    uint32_t loop_duration = current_time - last_loop_time;
    uint32_t target_loop_time = 1000000 / current_params.LOOP_FREQUENCY;
    if (loop_duration < target_loop_time) {
      delayMicroseconds(target_loop_time - loop_duration);
    }
  }
  last_loop_time = micros();
}

// 实时数据监控
void printKinematicData(const KinematicState &state, float correction) {
  if (!report_enabled) return;
  
  uart.print("IN_STEER:");
  uart.print((int)steering_pwm);
  uart.print(" IN_GAIN:");
  uart.print((int)gain_pwm);
  uart.print(" OUT_SERVO:");
  uart.print((int)PWMOutput::read(SERVO_OUT_CHANNEL));
  
  uart.print(" | Accel:(");
  uart.print(state.accel_x, 2);
  uart.print(",");
  uart.print(state.accel_y, 2);
  uart.print(")g");
  
  uart.print(" | ω:");
  uart.print(state.angular_vel, 1);
  uart.print("dps");
  
  uart.print(" | Correction:");
  uart.print(correction, 3);
  
  uart.println();
}

// EEPROM相关函数
void initializeDefaultParams() {
    current_params.BOARD_ROTATION = DEFAULT_BOARD_ROTATION;
    current_params.K_GAIN = DEFAULT_K_GAIN;
    current_params.DEFAULT_GAIN = DEFAULT_DEFAULT_GAIN;
    current_params.STEER_BY_ACC_RATE = DEFAULT_STEER_BY_ACC_RATE;
    current_params.COUNTER_STEER_RANGE = DEFAULT_COUNTER_STEER_RANGE;
    current_params.SERVO_LIMIT_LEFT = DEFAULT_SERVO_LIMIT_LEFT;
    current_params.SERVO_LIMIT_RIGHT = DEFAULT_SERVO_LIMIT_RIGHT;
    current_params.LOOP_FREQUENCY = DEFAULT_LOOP_FREQUENCY;
    current_params.IMU_FILTER = DEFAULT_IMU_FILTER;
    current_params.SERVO_FILTER = DEFAULT_SERVO_FILTER;
    current_params.ANGACC_FILTER = DEFAULT_ANGACC_FILTER;
    current_params.STEER_BY_ANGACC_RATE = DEFAULT_STEER_BY_ANGACC_RATE;
}

bool validateParamRange(uint8_t param_index, float value) {
    if (param_index >= sizeof(param_ranges)/sizeof(ParamRange)) return false;
    
    ParamRange range = param_ranges[param_index];
    return (value >= range.min && value <= range.max);
}

void loadParamsFromEEPROM() {
    SystemParams stored_params;
    
    // 尝试从EEPROM读取参数
    if (eeprom.readBlock(0, &stored_params, sizeof(SystemParams))) {
        // 验证每个参数的范围
        bool all_valid = true;
        for (uint8_t i = 0; i < sizeof(param_ranges)/sizeof(ParamRange); i++) {
            float* param_ptr = ((float*)&stored_params) + i;
            if (!validateParamRange(i, *param_ptr)) {
                all_valid = false;
                break;
            }
        }
        
        if (all_valid) {
            // 所有参数都有效，使用EEPROM中的值
            memcpy(&current_params, &stored_params, sizeof(SystemParams));
            uart.println("Parameters loaded from EEPROM");
            return;
        }
    }
    
    // EEPROM中没有有效数据或数据无效，使用默认值
    uart.println("Using default parameters");
    initializeDefaultParams();
    saveParamsToEEPROM();
}

void saveParamsToEEPROM() {
    eeprom.writeBlock(0, &current_params, sizeof(SystemParams));
    uart.println("Parameters saved to EEPROM");
}

// 串口命令处理
void processSerialCommand() {
    if (serial_index == 0) return;
    
    serial_buffer[serial_index] = '\0'; // 确保字符串以null结尾
    
    // 解析命令
    char* token = strtok(serial_buffer, " ");
    
    if (token == NULL) return;
    
    if (strcmp(token, "report") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            if (strcmp(token, "on") == 0) {
                report_enabled = true;
                uart.println("Report enabled");
            } else if (strcmp(token, "off") == 0) {
                report_enabled = false;
                uart.println("Report disabled");
            }
        }
    }
    else if (strcmp(token, "get") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            // 查找参数名
            for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
                if (strcmp(token, param_names[i]) == 0) {
                    float value = *((float*)&current_params + i);
                    uart.print(param_names[i]);
                    uart.print(" = ");
                    uart.println(value, 4);
                    return;
                }
            }
            uart.print("Unknown parameter: ");
            uart.println(token);
        }
    }
    else if (strcmp(token, "set") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            char* param_name = token;
            token = strtok(NULL, " ");
            if (token != NULL) {
                float new_value = atof(token);
                
                // 查找参数名
                for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
                    if (strcmp(param_name, param_names[i]) == 0) {
                        if (validateParamRange(i, new_value)) {
                            *((float*)&current_params + i) = new_value;
                            saveParamsToEEPROM();
                            
                            uart.print("Set ");
                            uart.print(param_names[i]);
                            uart.print(" to ");
                            uart.println(new_value, 4);
                            
                            // 重启后生效的提示
                            uart.println("Note: Some parameters require restart to take effect");
                        } else {
                            uart.print("Value out of range! Valid range: ");
                            uart.print(param_ranges[i].min);
                            uart.print(" to ");
                            uart.println(param_ranges[i].max);
                        }
                        return;
                    }
                }
                uart.print("Unknown parameter: ");
                uart.println(param_name);
            }
        }
    }
    else if (strcmp(token, "help") == 0) {
        uart.println("Available commands:");
        uart.println("  report on/off - Enable/disable data reporting");
        uart.println("  get <param>   - Get parameter value");
        uart.println("  set <param> <value> - Set parameter value");
        uart.println("  help          - Show this help");
        uart.println("  params        - List all parameters");
    }
    else if (strcmp(token, "params") == 0) {
        uart.println("Available parameters:");
        for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
            float value = *((float*)&current_params + i);
            uart.print("  ");
            uart.print(param_names[i]);
            uart.print(" = ");
            uart.println(value, 4);
        }
    }
    else {
        uart.print("Unknown command: ");
        uart.println(token);
        uart.println("Type 'help' for available commands");
    }
}

void processSerialInput() {
    while (uart.available() > 0) {
        char c = uart.read();
        
        if (c == '\r' || c == '\n') {
            if (serial_index > 0) {
                processSerialCommand();
                serial_index = 0;
            }
        } else if (serial_index < sizeof(serial_buffer) - 1) {
            serial_buffer[serial_index++] = c;
        }
    }
}

void setup() {
    // 初始化UART
    uart.begin(115200);
    
    // 初始化EEPROM
    eeprom.init();
    
    // 从EEPROM加载参数
    loadParamsFromEEPROM();
    
    // 初始化MPU6000
    if (!mpu.init()) {
        uart.println("ERROR: MPU6000 initialization failed!");
        while(1) {}
    }
    
    // 初始化PWM输出系统
    PWMOutput::init();
    PWMOutput::enable(SERVO_OUT_CHANNEL);
    PWMOutput::setFrequency(50);
    
    // 设置输入引脚和中断
    pinMode(STEERING_IN_PIN, INPUT);
    pinMode(GAIN_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), steeringISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GAIN_IN_PIN), gainISR, CHANGE);
    
    // 初始化滤波器
    accel_x_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    accel_y_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    gyro_z_filter.setCutoffFrequency(2*current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    servo_filter.setCutoffFrequency(current_params.SERVO_FILTER, current_params.LOOP_FREQUENCY);
    
    // 初始化时间戳
    steering_last_update = millis();
    gain_last_update = millis();
    
    mpu.delay_ms(100);
    
    uart.println("APM2.8 RC陀螺仪系统 - 使用输出1通道(CH1)");
    uart.print("飞控板安装方向: ");
    uart.print((int)current_params.BOARD_ROTATION);
    uart.println("度");
    uart.println("舵机连接到引脚12");
    uart.println("Serial commands: report on/off, get/set <param>, help");
    uart.println("Data reporting is OFF by default");
}

void loop() {
    uint32_t loop_start_time = micros();
    
    // 处理串口输入
    processSerialInput();
    
    checkInputTimeout();
    
    if (mpu.read_sensors()) {
        float accel_x_raw = mpu.get_accel_x();
        float accel_y_raw = mpu.get_accel_y();
        float gyro_z_raw = mpu.get_gyro_z();
        
        float accel_x_filt, accel_y_filt, gyro_z_filt;
        applySensorFilters(accel_x_raw, accel_y_raw, gyro_z_raw, 
                          accel_x_filt, accel_y_filt, gyro_z_filt);
        
        KinematicState state;
        // 调用时传入旋转角度参数
        calculateKinematicState(accel_x_filt, accel_y_filt, gyro_z_filt, state, current_params.BOARD_ROTATION-90);
        
        float gain = calculateGain();
        float correction = calculateCounterSteerByKinematics(state, gain);
        
        outputServo(steering_pwm, correction);
        
        static uint32_t last_print = 0;
        if (millis() - last_print > 100) {
            printKinematicData(state, correction);
            last_print = millis();
        }
    } else {
        uart.println("ERROR: Failed to read MPU6000 sensor data!");
    }
    
    controlLoopFrequency();
}