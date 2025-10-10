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
    float GYRO_EXP; 
    float OUTPUT_EXP; 
    float STEER_BY_ANGVEL_RATE;
    float STEER_BY_ANG_RATE;
    float STEER_BY_ANG_LIMIT;
    float ANGVEL_ZERO;
};

// 默认参数值
#define DEFAULT_BOARD_ROTATION 270
#define DEFAULT_K_GAIN 0.0134
#define DEFAULT_DEFAULT_GAIN 200
#define DEFAULT_STEER_BY_ACC_RATE 1.5
#define DEFAULT_COUNTER_STEER_RANGE 0.95
#define DEFAULT_SERVO_LIMIT_LEFT 1
#define DEFAULT_SERVO_LIMIT_RIGHT 1
#define DEFAULT_LOOP_FREQUENCY 100
#define DEFAULT_IMU_FILTER 30
#define DEFAULT_SERVO_FILTER 30
#define DEFAULT_ANGACC_FILTER 20
#define DEFAULT_STEER_BY_ANGACC_RATE 1
#define DEFAULT_GYRO_EXP -0.18
#define DEFAULT_OUTPUT_EXP 0
#define DEFAULT_STEER_BY_ANGVEL_RATE 0.9
#define DEFAULT_STEER_BY_ANG_RATE 1.55
#define DEFAULT_STEER_BY_ANG_LIMIT 40
#define DEFAULT_ANGVEL_ZERO 0.0

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
    {0, 10},        // STEER_BY_ANGACC_RATE
    {-1, 1},        // GYRO_EXP
    {-1, 1},         // OUTPUT_EXP
    {0, 20},        // STEER_BY_ANGVEL_RATE
    {0, 20},        // STEER_BY_ANG_RATE
    {10, 90},      // STEER_BY_ANG_LIMIT
    {-20, 20}       // ANGVEL_ZERO
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
    "STEER_BY_ANGACC_RATE",
    "GYRO_EXP",
    "OUTPUT_EXP",
    "STEER_BY_ANGVEL_RATE",
    "STEER_BY_ANG_RATE",
    "STEER_BY_ANG_LIMIT",
    "ANGVEL_ZERO"
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
  // 规范化角度到0-359度范围
  rotation_deg = rotation_deg % 360;
  if (rotation_deg < 0) {
    rotation_deg += 360;
  }
  
  // 如果角度是0度，无需旋转
  if (rotation_deg == 0) {
    return;
  }
  
  // 将角度转换为弧度
  float theta = rotation_deg * M_PI / 180.0;
  
  // 计算旋转矩阵（2D平面旋转）
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  
  // 应用旋转矩阵到加速度数据
  float temp_accel_x = accel_x * cos_theta - accel_y * sin_theta;
  float temp_accel_y = accel_x * sin_theta + accel_y * cos_theta;
  
  // 更新加速度值
  accel_x = temp_accel_x;
  accel_y = temp_accel_y;
  
  // 注意：角速度方向不变（绕Z轴旋转）
  // gyro_z保持不变
}

// S型曲线处理函数
float applySCurve(float input, float exp_param, bool use_limits = false, float limit_positive = 1.0, float limit_negative = 1.0) {
    if (fabs(input) < 0.001f) return 0.0f; // 处理零输入
    
    // 当a接近0时，使用线性函数
    if (fabs(exp_param) < 0.001f) {
        float result = input;
        if (use_limits) {
            if (input > 0) {
                result *= limit_positive;
            } else {
                result *= limit_negative;
            }
        }
        return result;
    }
    
    float abs_x = fabs(input);
    float sign_x = (input > 0) ? 1.0f : -1.0f;
    
    // 计算100^a
    float base_pow_a = pow(100.0f, exp_param);
    
    // 计算分子部分：((100^a)^(|x|) / (100^a)) - (1/(100^a))
    float numerator = (pow(base_pow_a, abs_x) / base_pow_a) - (1.0f / base_pow_a);
    
    // 计算分母部分：1 - (1/(100^a))
    float denominator = 1.0f - (1.0f / base_pow_a);
    
    // 最终结果
    float result = sign_x * (numerator / denominator);
    
    // 应用边界限制
    if (use_limits) {
        if (input > 0) {
            result *= limit_positive;
        } else {
            result *= limit_negative;
        }
    }
    
    return result;
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

// 修改后的反打控制函数
float calculateCounterSteerByKinematics(const KinematicState &state, float gain) {
    // 参数定义
    const float ACCEL_GAIN = current_params.STEER_BY_ACC_RATE;     // 横向加速度增益系数
    const float GYRO_GAIN = current_params.STEER_BY_ANGVEL_RATE;   // 角速度增益系数 - 改为可调参数
    const float ANGACC_GAIN = current_params.STEER_BY_ANGACC_RATE; // 角加速度增益系数
    const float DEADBAND_ACCEL = 0.2f; // 加速度死区 (g)
    const float DEADBAND_GYRO = 0.8f;  // 角速度死区 (度/秒)
    const uint32_t DEADBAND_TIMEOUT_MS = 250; // 角速度死区超时时间(ms)
    
    static float angle_integral = 0.0f; // 角度积分器
    static uint32_t last_gyro_active_time = 0; // 上次角速度活跃时间
    static bool first_run = true; // 首次运行标志
    
    float counter_steer = 0.0f;
    
    // 获取当前时间
    uint32_t current_time = millis();
    
    // 初始化时间戳
    if (first_run) {
        last_gyro_active_time = current_time;
        first_run = false;
    }
    
    // 应用角速度零偏校准
    float calibrated_angular_vel = state.angular_vel + current_params.ANGVEL_ZERO;
    
    //基于横向加速度的反打分量
    if (fabs(state.accel_x) > DEADBAND_ACCEL) {
        // 加速度与反打方向相反
        float accel_component = state.accel_x * ACCEL_GAIN;
        counter_steer += accel_component;
    }
    
    //基于角速度的反打分量
    if (fabs(calibrated_angular_vel) > DEADBAND_GYRO) {
        // 更新角速度活跃时间
        last_gyro_active_time = current_time;
        
        // 角速度与反打方向相同
        float gyro_component = calibrated_angular_vel * GYRO_GAIN;
        counter_steer += gyro_component;
        
        //基于角加速度的反打预测
        float angacc_component = current_params.ANGACC_FILTER * (calibrated_angular_vel - angular_accel_integral);
        angular_accel_integral += angacc_component;
        angacc_component *= ANGACC_GAIN / current_params.LOOP_FREQUENCY;
        //counter_steer += angacc_component;
        
        // 角度积分项 - 只在未达到饱和限制时积分
        float angle_increment = calibrated_angular_vel;
        
        // 检查积分饱和限制
        if (fabs(angle_integral + angle_increment) <= current_params.STEER_BY_ANG_LIMIT*current_params.LOOP_FREQUENCY) {
            angle_integral += angle_increment;
        }

        //去除循环周期影响
        float real_angle = angle_integral/ current_params.LOOP_FREQUENCY;
        float angle_component = real_angle * current_params.STEER_BY_ANG_RATE;
        counter_steer += angle_component;
    }
    else {
        // 角速度在死区内，检查是否需要清零积分器
        if (current_time - last_gyro_active_time > DEADBAND_TIMEOUT_MS) {
            angle_integral = 0.0f;
        }
        float real_angle = angle_integral/ current_params.LOOP_FREQUENCY;
        float angle_component = real_angle * current_params.STEER_BY_ANG_RATE;
        counter_steer += angle_component;
    }
    //应用感度调节
    counter_steer *= gain * current_params.K_GAIN;
    
    // 应用陀螺仪输出的S型曲线
    if (fabs(current_params.GYRO_EXP) > 0.001f) {
        // 归一化到[-1,1]范围进行处理
        float normalized_steer = counter_steer / current_params.COUNTER_STEER_RANGE;
        normalized_steer = applySCurve(normalized_steer, current_params.GYRO_EXP);
        counter_steer = normalized_steer * current_params.COUNTER_STEER_RANGE;
    }
    
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
    
    // 应用舵机输出的S型曲线
    if (fabs(current_params.OUTPUT_EXP) > 0.001f) {
        final_output = applySCurve(final_output, current_params.OUTPUT_EXP, true, 
                                 current_params.SERVO_LIMIT_RIGHT, current_params.SERVO_LIMIT_LEFT);
    } else {
        // 如果没有S曲线，使用线性限制
        final_output = constrain(final_output, -current_params.SERVO_LIMIT_LEFT, current_params.SERVO_LIMIT_RIGHT);
    }
    
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
  
  float calibrated_angular_vel = state.angular_vel + current_params.ANGVEL_ZERO;
  
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
  uart.print(calibrated_angular_vel, 1);
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
    current_params.GYRO_EXP = DEFAULT_GYRO_EXP;
    current_params.OUTPUT_EXP = DEFAULT_OUTPUT_EXP;
    current_params.STEER_BY_ANGVEL_RATE = DEFAULT_STEER_BY_ANGVEL_RATE;
    current_params.STEER_BY_ANG_RATE = DEFAULT_STEER_BY_ANG_RATE;
    current_params.STEER_BY_ANG_LIMIT = DEFAULT_STEER_BY_ANG_LIMIT;
    current_params.ANGVEL_ZERO = DEFAULT_ANGVEL_ZERO;
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
        calculateKinematicState(accel_x_filt, accel_y_filt, gyro_z_filt, state, current_params.BOARD_ROTATION);
        
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