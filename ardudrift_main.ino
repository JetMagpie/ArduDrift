#include "MPU6000.h"
#include "UART.h"
#include "PWMOutput.h"
#include <math.h>

//调参参数
#define BOARD_ROTATION 90 // 飞控板安装方向 (0度=标准, 90度=顺时针旋转90度)
#define K_GAIN 0.015 //感度乘数，决定遥控器调整感度的范围
#define DEFAULT_GAIN 200 //不连接感度通道时的默认感度
#define STEER_BY_ACC_RATE 3 //侧滑加速度提供的反打和角速度提供的反打的比例
#define COUNTER_STEER_RANGE 0.9 //反打量范围，用于调整反打动作的优先级
#define SERVO_LIMIT_LEFT 1
#define SERVO_LIMIT_RIGHT 0.55 //舵机左右范围（最大为1）
#define LOOP_FREQUENCY 300 //控制频率
#define IMU_FILTER 10//传感器滤波截止频率，即灵敏度，越大越灵敏
#define SERVO_FILTER 15//舵机信号滤波器，即防抖，越小越防抖
#define ANGACC_FILTER 10//角加速度的滤波器
#define STEER_BY_ANGACC_RATE 1//角加速度参与反打的比例

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
  //float curvature_radius, tangent_direction;
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
      uart.println(rotation_deg);
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
    gain_pwm = PWM_NEUTRAL+DEFAULT_GAIN;
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
  const float ACCEL_GAIN = STEER_BY_ACC_RATE;     // 横向加速度增益系数
  const float GYRO_GAIN = 1.0f;     // 角速度增益系数
  const float ANGACC_GAIN = STEER_BY_ANGACC_RATE;   //角加速度增益系数
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
    float angacc_component = ANGACC_FILTER * (state.angular_vel-angular_accel_integral);
    angular_accel_integral += angacc_component;
    angacc_component *= ANGACC_GAIN/LOOP_FREQUENCY;
    //counter_steer += angacc_component;
  }
  //应用感度调节
  counter_steer *= gain*K_GAIN;
  //限制输出范围
  counter_steer = constrain(counter_steer, -COUNTER_STEER_RANGE, COUNTER_STEER_RANGE);
  
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
  final_output = constrain(final_output, -SERVO_LIMIT_LEFT, SERVO_LIMIT_RIGHT);
  
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
    uint32_t target_loop_time = 1000000 / LOOP_FREQUENCY;
    if (loop_duration < target_loop_time) {
      delayMicroseconds(target_loop_time - loop_duration);
    }
  }
  last_loop_time = micros();
}

// 实时数据监控
void printKinematicData(const KinematicState &state, float correction) {
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

void setup() {
  // 初始化UART
  uart.begin(115200);
  
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
  accel_x_filter.setCutoffFrequency(IMU_FILTER, LOOP_FREQUENCY);
  accel_y_filter.setCutoffFrequency(IMU_FILTER, LOOP_FREQUENCY);
  gyro_z_filter.setCutoffFrequency(2*IMU_FILTER, LOOP_FREQUENCY);
  servo_filter.setCutoffFrequency(SERVO_FILTER, LOOP_FREQUENCY);
  
  // 初始化时间戳
  steering_last_update = millis();
  gain_last_update = millis();
  
  mpu.delay_ms(100);
  
  uart.println("APM2.8 RC陀螺仪系统 - 使用输出1通道(CH1)");
  uart.print("飞控板安装方向: ");
  uart.print(BOARD_ROTATION);
  uart.println("度");
  uart.println("舵机连接到引脚12");
}

void loop() {
  uint32_t loop_start_time = micros();
  
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
    calculateKinematicState(accel_x_filt, accel_y_filt, gyro_z_filt, state, BOARD_ROTATION-90);
    
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