#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>
#include <string.h>
#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>

#include <nrf_nvmc.h> 

// 参数结构体
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
    float ANG_HALF_LIFE;
};

// 默认参数值
#define DEFAULT_BOARD_ROTATION 270
#define DEFAULT_K_GAIN 0.003
#define DEFAULT_DEFAULT_GAIN 200
#define DEFAULT_STEER_BY_ACC_RATE 0.5
#define DEFAULT_COUNTER_STEER_RANGE 0.95
#define DEFAULT_SERVO_LIMIT_LEFT 1
#define DEFAULT_SERVO_LIMIT_RIGHT 1
#define DEFAULT_LOOP_FREQUENCY 100
#define DEFAULT_IMU_FILTER 30
#define DEFAULT_SERVO_FILTER 120
#define DEFAULT_ANGACC_FILTER 30
#define DEFAULT_STEER_BY_ANGACC_RATE 1
#define DEFAULT_GYRO_EXP -0.18
#define DEFAULT_OUTPUT_EXP 0
#define DEFAULT_STEER_BY_ANGVEL_RATE 1.1
#define DEFAULT_STEER_BY_ANG_RATE 1.0
#define DEFAULT_STEER_BY_ANG_LIMIT 90
#define DEFAULT_ANGVEL_ZERO 0.0
#define DEFAULT_ANG_HALF_LIFE 0.15

SystemParams current_params;

// 全局变量
volatile bool report_enabled = false;
char serial_buffer[64];
uint8_t serial_index = 0;

// 创建LSM6DS3实例
LSM6DS3 imu(I2C_MODE, 0x6A);

//imu去毛刺
#define MEDIAN_WINDOW 3
float gyro_buffer[MEDIAN_WINDOW];
uint8_t gyro_idx = 0;

// nRF52840引脚定义
#define STEERING_IN_PIN 4    // P0.04
#define GAIN_IN_PIN 5        // P0.05
#define SERVO_OUT_PIN 3      // P0.03

// PWM信号设定
#define PWM_MIN 800
#define PWM_MAX 2200
#define PWM_NEUTRAL 1500
#define INPUT_TIMEOUT_MS 500

// PWM输出频率设置
#define PWM_FREQUENCY 100
#define PWM_CLOCK_FREQ 1000000
#define PWM_RESOLUTION 32768

// 硬件定时器用于精确时间测量
#define TIMER_PRESCALER 4  // 16MHz / 2^4 = 1MHz (1µs分辨率)
static volatile uint32_t timer_overflow_count = 0;

#define FLASH_START_ADDR 0x3E000  // 可用闪存末尾区域

// nRF52840 PWM输出实例
static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t m_seq_values;
static nrf_pwm_sequence_t m_seq;
static uint16_t m_pwm_period;

// 舵机输出值记录
static uint16_t last_servo_output = PWM_NEUTRAL;

// 参数验证范围结构
struct ParamRange {
    float min;
    float max;
};

// 参数验证范围
const ParamRange param_ranges[] = {
    {0, 360},       // BOARD_ROTATION
    {-0.1, 0.1},   // K_GAIN
    {0, 500},      // DEFAULT_GAIN
    {0, 20},       // STEER_BY_ACC_RATE
    {0, 1.0},      // COUNTER_STEER_RANGE
    {0, 1.0},      // SERVO_LIMIT_LEFT
    {0, 1.0},      // SERVO_LIMIT_RIGHT
    {50, 1000},    // LOOP_FREQUENCY
    {1, 500},      // IMU_FILTER
    {1, 500},      // SERVO_FILTER
    {1, 500},      // ANGACC_FILTER
    {0, 10},       // STEER_BY_ANGACC_RATE
    {-1, 1},       // GYRO_EXP
    {-1, 1},       // OUTPUT_EXP
    {0, 20},       // STEER_BY_ANGVEL_RATE
    {0, 20},       // STEER_BY_ANG_RATE
    {10, 90},      // STEER_BY_ANG_LIMIT
    {-20, 20},     // ANGVEL_ZERO
    {0.001,2}      // ANG_HALF_LIFE
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
    "ANGVEL_ZERO",
    "ANG_HALF_LIFE"
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

// 坐标旋转函数
void rotateSensorData(float &accel_x, float &accel_y, float &gyro_z, int rotation_deg) {
  rotation_deg = rotation_deg % 360;
  if (rotation_deg < 0) {
    rotation_deg += 360;
  }
  
  if (rotation_deg == 0) {
    return;
  }
  
  float theta = rotation_deg * M_PI / 180.0;
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  
  float temp_accel_x = accel_x * cos_theta - accel_y * sin_theta;
  float temp_accel_y = accel_x * sin_theta + accel_y * cos_theta;
  
  accel_x = temp_accel_x;
  accel_y = temp_accel_y;
}

// S型曲线处理函数
float applySCurve(float input, float exp_param, bool use_limits = false, float limit_positive = 1.0, float limit_negative = 1.0) {
    if (fabs(input) < 0.001f) return 0.0f;
    
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
    
    float base_pow_a = pow(100.0f, exp_param);
    float numerator = (pow(base_pow_a, abs_x) / base_pow_a) - (1.0f / base_pow_a);
    float denominator = 1.0f - (1.0f / base_pow_a);
    
    float result = sign_x * (numerator / denominator);
    
    if (use_limits) {
        if (input > 0) {
            result *= limit_positive;
        } else {
            result *= limit_negative;
        }
    }
    
    return result;
}

//去毛刺函数
float median_filter(float input) {
    gyro_buffer[gyro_idx] = input;
    gyro_idx = (gyro_idx + 1) % MEDIAN_WINDOW;
    
    // 复制并排序（简单冒泡）
    float sorted[MEDIAN_WINDOW];
    memcpy(sorted, gyro_buffer, sizeof(sorted));
    for (int i = 0; i < MEDIAN_WINDOW-1; i++) {
        for (int j = i+1; j < MEDIAN_WINDOW; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
    return sorted[MEDIAN_WINDOW/2];
}

//PWM 输入捕获
//Steering
volatile uint16_t steering_pulse_width_us = PWM_NEUTRAL;
volatile uint32_t steering_last_capture_time = 0;

//Gain
volatile uint16_t gain_pulse_width_us = PWM_NEUTRAL;
volatile uint32_t gain_last_capture_time = 0;

//TIMER 初始化
static void timer_init(NRF_TIMER_Type *timer)
{
    timer->MODE      = TIMER_MODE_MODE_Timer;
    timer->PRESCALER = 4;                 // 1 MHz
    timer->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
    timer->TASKS_CLEAR = 1;
    timer->TASKS_START = 1;
}

//初始化 PWM 捕获
bool initPWMCapture()
{
    pinMode(STEERING_IN_PIN, INPUT_PULLDOWN);
    pinMode(GAIN_IN_PIN, INPUT_PULLDOWN);

    //初始化 TIMER2（用于两路捕获）
    timer_init(NRF_TIMER2);   // 1MHz, 32位

    //GPIOTE 配置
    NRF_GPIOTE->CONFIG[0] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (STEERING_IN_PIN << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[1] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (STEERING_IN_PIN << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[2] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (GAIN_IN_PIN << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[3] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (GAIN_IN_PIN << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    //PPI指向 TIMER2
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER2->TASKS_CAPTURE[0];   // STEERING 上升沿

    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_TIMER2->TASKS_CAPTURE[1];   // STEERING 下降沿

    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[2];
    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_TIMER2->TASKS_CAPTURE[2];   // GAIN 上升沿

    NRF_PPI->CH[3].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[3];
    NRF_PPI->CH[3].TEP = (uint32_t)&NRF_TIMER2->TASKS_CAPTURE[3];   // GAIN 下降沿

    // 使能 PPI 通道 0~3
    NRF_PPI->CHENSET = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);

    Serial.println("PWM capture initialized");
    return true;
}

//PWM 读取
uint16_t getSteeringPWM()
{
    uint32_t rise = NRF_TIMER2->CC[2];
    uint32_t fall = NRF_TIMER2->CC[3];

    if (fall > rise) {  // 有效脉冲
        uint32_t pw = fall - rise;
        if (pw >= PWM_MIN && pw <= PWM_MAX) {
            steering_pulse_width_us = pw;
            steering_last_capture_time = millis();
        }
    }
    return steering_pulse_width_us;
}

uint16_t getGainPWM() {
    uint32_t rise = NRF_TIMER2->CC[0];
    uint32_t fall = NRF_TIMER2->CC[1];

    if (fall > rise) {
        uint32_t pw = fall - rise;
        if (pw >= PWM_MIN && pw <= PWM_MAX) {
            gain_pulse_width_us = pw;
            gain_last_capture_time = millis();
        }
    }
    return gain_pulse_width_us;
}


// 检查输入超时
void checkInputTimeout() {
    uint32_t current_time = millis();
    if (current_time - steering_last_capture_time > INPUT_TIMEOUT_MS) {
        steering_pulse_width_us = PWM_NEUTRAL;
    }
    if (current_time - gain_last_capture_time > INPUT_TIMEOUT_MS) {
        gain_pulse_width_us = PWM_NEUTRAL + current_params.DEFAULT_GAIN;
    }
}

// 应用传感器滤波器
void applySensorFilters(float accel_x_raw, float accel_y_raw, float gyro_z_raw, 
                       float &accel_x_filt, float &accel_y_filt, float &gyro_z_filt) {
    accel_x_filt = accel_x_filter.update(accel_x_raw);
    accel_y_filt = accel_y_filter.update(accel_y_raw);
    gyro_z_filt = gyro_z_filter.update(gyro_z_raw);
}

// 计算运动学状态
void calculateKinematicState(float accel_x, float accel_y, float angular_vel, 
                            KinematicState &state, int rotation_deg = 0) {
    rotateSensorData(accel_x, accel_y, angular_vel, rotation_deg);
    
    state.accel_x = accel_x;
    state.accel_y = accel_y;
    state.angular_vel = angular_vel;
    
    state.total_accel = sqrt(accel_x * accel_x + accel_y * accel_y);
    state.accel_direction = atan2(accel_y, accel_x) * 180.0 / M_PI;
    
    float angular_vel_rad = fabs(angular_vel) * M_PI / 180.0;
    float total_accel_ms2 = state.total_accel * 9.81;
}

// 反打控制函数
float calculateCounterSteerByKinematics(const KinematicState &state, float gain) {
    const float ACCEL_GAIN = 10*current_params.STEER_BY_ACC_RATE;
    const float GYRO_GAIN = current_params.STEER_BY_ANGVEL_RATE;
    const float ANGACC_GAIN = current_params.STEER_BY_ANGACC_RATE;
    const float REDUCTION = pow(2.0, -1.0 / (current_params.LOOP_FREQUENCY*current_params.ANG_HALF_LIFE));
    const float DEADBAND_ACCEL = 0.2f;
    const float DEADBAND_GYRO = 0.8f;
    const uint32_t DEADBAND_TIMEOUT_MS = 250;
    
    static float angle_integral = 0.0f;
    static uint32_t last_gyro_active_time = 0;
    static bool first_run = true;
    static float angular_accel_integral = 0.0f;
    
    float counter_steer = 0.0f;
    uint32_t current_time = millis();
    
    if (first_run) {
        last_gyro_active_time = current_time;
        first_run = false;
    }
    
    float calibrated_angular_vel = state.angular_vel + current_params.ANGVEL_ZERO;
    
    if (fabs(calibrated_angular_vel) > DEADBAND_GYRO) {
        last_gyro_active_time = current_time;
        
        float gyro_component = calibrated_angular_vel * GYRO_GAIN;
        counter_steer += gyro_component;
        
        float angacc_component = current_params.ANGACC_FILTER * (calibrated_angular_vel - angular_accel_integral);
        angular_accel_integral += angacc_component;
        angacc_component *= ANGACC_GAIN / current_params.LOOP_FREQUENCY;

        float angle_increment = calibrated_angular_vel;
        if (fabs(state.accel_x) > DEADBAND_ACCEL) {
            float accel_component = state.accel_x * ACCEL_GAIN;
            angle_increment -= accel_component;
        }

        if (fabs(angle_integral + angle_increment) <= current_params.STEER_BY_ANG_LIMIT*current_params.LOOP_FREQUENCY) {
            angle_integral += angle_increment;
            angle_integral *= REDUCTION;
        }

        float real_angle = angle_integral/ current_params.LOOP_FREQUENCY;
        float angle_component = real_angle * current_params.STEER_BY_ANG_RATE;
        counter_steer += angle_component;
    }
    else {
        if (current_time - last_gyro_active_time > DEADBAND_TIMEOUT_MS) {
            angle_integral *= REDUCTION;
        }
        float real_angle = angle_integral/ current_params.LOOP_FREQUENCY;
        float angle_component = real_angle * current_params.STEER_BY_ANG_RATE;
        counter_steer += angle_component;
    }
    
    counter_steer *= gain * current_params.K_GAIN;
    
    if (fabs(current_params.GYRO_EXP) > 0.001f) {
        float normalized_steer = counter_steer / current_params.COUNTER_STEER_RANGE;
        normalized_steer = applySCurve(normalized_steer, current_params.GYRO_EXP);
        counter_steer = normalized_steer * current_params.COUNTER_STEER_RANGE;
    }
    
    counter_steer = constrain(counter_steer, -current_params.COUNTER_STEER_RANGE, current_params.COUNTER_STEER_RANGE);
    
    return counter_steer;
}

// 计算感度系数
float calculateGain() {
    uint16_t gain_pwm = getGainPWM();
    if (gain_pwm < PWM_MIN || gain_pwm > PWM_MAX) {
        return 0.5;
    }
    float normalized = (float)(gain_pwm - 1500) / 500.0;
    normalized = constrain(normalized, 0.0, 1.0);
    return normalized;
}

// 初始化nRF52840的硬件PWM输出
bool initPWMOutput() {
    m_pwm_period = PWM_CLOCK_FREQ / PWM_FREQUENCY;   // 10000

    nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG;
    config.output_pins[0] = SERVO_OUT_PIN;
    config.base_clock = NRF_PWM_CLK_1MHz;
    config.top_value = m_pwm_period;
    config.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    config.step_mode = NRF_PWM_STEP_AUTO;

    if (nrfx_pwm_init(&m_pwm0, &config, NULL) != NRFX_SUCCESS) {
        Serial.println("PWM init failed");
        return false;
    }

    m_seq_values.channel_0 = PWM_NEUTRAL;
    m_seq_values.channel_1 = 0;
    m_seq_values.channel_2 = 0;
    m_seq_values.channel_3 = 0;

    m_seq.values.p_individual = &m_seq_values;
    m_seq.length = NRF_PWM_VALUES_LENGTH(m_seq_values);
    m_seq.repeats = 0;
    m_seq.end_delay = 0;

    // 只启动一次
    nrfx_pwm_simple_playback(&m_pwm0, &m_seq, 1, NRFX_PWM_FLAG_LOOP);

    Serial.println("PWM output initialized at 100Hz");
    return true;
}


// 设置PWM脉宽（微秒）
void setPulseWidth(uint16_t pulse_us) {
    pulse_us = constrain(pulse_us, PWM_MIN, PWM_MAX);

    // 1us = 1 tick（1MHz clock）
    m_seq_values.channel_0 = m_pwm_period - pulse_us;

    last_servo_output = pulse_us;
}

// 输出到舵机
void outputServo(float steering_input, float correction) {
    if (steering_input < PWM_MIN || steering_input > PWM_MAX) {
        steering_input = PWM_NEUTRAL;
    }
    
    float normalized_input = (steering_input - 1500) / 500.0;
    float final_output = normalized_input + correction;
    
    if (fabs(current_params.OUTPUT_EXP) > 0.001f) {
        final_output = applySCurve(final_output, current_params.OUTPUT_EXP, true, 
                                 current_params.SERVO_LIMIT_RIGHT, current_params.SERVO_LIMIT_LEFT);
    } else {
        final_output = constrain(final_output, -current_params.SERVO_LIMIT_LEFT, current_params.SERVO_LIMIT_RIGHT);
    }
    
    int pwm_output_raw = PWM_NEUTRAL + (int)(final_output * 500);
    pwm_output_raw = constrain(pwm_output_raw, PWM_MIN, PWM_MAX);
    
    float normalized_filtered = servo_filter.update((pwm_output_raw - 1500) / 500.0);
    uint16_t filtered_pwm = PWM_NEUTRAL + (int)(normalized_filtered * 500);
    
    setPulseWidth(filtered_pwm);
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
    
    uint16_t steering_pwm = getSteeringPWM();
    uint16_t gain_pwm = getGainPWM();
    float calibrated_angular_vel = state.angular_vel + current_params.ANGVEL_ZERO;
    
    Serial.print("IN_STEER:");
    Serial.print((int)steering_pwm);
    Serial.print(" IN_GAIN:");
    Serial.print((int)gain_pwm);
    Serial.print(" OUT_SERVO:");
    Serial.print((int)last_servo_output);
    
    Serial.print(" | Accel:(");
    Serial.print(state.accel_x, 2);
    Serial.print(",");
    Serial.print(state.accel_y, 2);
    Serial.print(")g");
    
    Serial.print(" | ω:");
    Serial.print(calibrated_angular_vel, 1);
    Serial.print("dps");
    
    Serial.print(" | Correction:");
    Serial.print(correction, 3);
    
    Serial.println();
}

// 参数读写相关函数
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
    current_params.ANG_HALF_LIFE = DEFAULT_ANG_HALF_LIFE;
}

bool validateParamRange(uint8_t param_index, float value) {
    if (param_index >= sizeof(param_ranges)/sizeof(ParamRange)) return false;
    ParamRange range = param_ranges[param_index];
    return (value >= range.min && value <= range.max);
}

void saveParamsToFlash() {
    uint32_t* src = (uint32_t*)&current_params;
    uint32_t* dst = (uint32_t*)FLASH_START_ADDR;
    size_t words = sizeof(SystemParams)/4;

    //擦除页
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    nrf_nvmc_page_erase(FLASH_START_ADDR);
    while (NRF_NVMC->READY == 0) {}

    //写入数据
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    for (size_t i = 0; i < words; i++) {
        nrf_nvmc_write_word((uint32_t)&dst[i], src[i]);
        while (NRF_NVMC->READY == 0) {} // 等待写入完成
    }

    //回到只读模式
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;

    Serial.println("Parameters saved to Flash");
}

void loadParamsFromFlash() {
    //临时存储从 Flash 读取的数据
    SystemParams stored_params;

    //读取 Flash 数据
    memcpy(&stored_params, (void*)FLASH_START_ADDR, sizeof(SystemParams));

    bool all_valid = true;

    //遍历每个参数，验证范围
    for (uint8_t i = 0; i < sizeof(param_ranges)/sizeof(ParamRange); i++) {
        float* param_ptr = ((float*)&stored_params) + i;
        if (!validateParamRange(i, *param_ptr)) {
            Serial.print("Parameter ");
            Serial.print(param_names[i]);
            Serial.print(" out of range: ");
            Serial.println(*param_ptr);
            all_valid = false;
        }
    }

    if (all_valid) {
        //如果 Flash 中所有参数合法，则加载
        memcpy(&current_params, &stored_params, sizeof(SystemParams));
        Serial.println("Parameters loaded from Flash successfully");
    } else {
        //否则，使用默认值初始化
        Serial.println("Invalid or uninitialized Flash data detected. Using default parameters.");
        initializeDefaultParams();
        //保存到 Flash
        saveParamsToFlash();
        Serial.println("Default parameters applied. Saved to Flash.");
    }
}

//串口命令处理
void processSerialCommand() {
    if (serial_index == 0) return;
    
    serial_buffer[serial_index] = '\0';
    char* token = strtok(serial_buffer, " ");
    
    if (token == NULL) return;
    
    if (strcmp(token, "report") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            if (strcmp(token, "on") == 0) {
                report_enabled = true;
                Serial.println("Report enabled");
            } else if (strcmp(token, "off") == 0) {
                report_enabled = false;
                Serial.println("Report disabled");
            }
        }
    }
    else if (strcmp(token, "get") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
                if (strcmp(token, param_names[i]) == 0) {
                    float value = *((float*)&current_params + i);
                    Serial.print(param_names[i]);
                    Serial.print(" = ");
                    Serial.println(value, 4);
                    return;
                }
            }
            Serial.print("Unknown parameter: ");
            Serial.println(token);
        }
    }
    else if (strcmp(token, "set") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            char* param_name = token;
            token = strtok(NULL, " ");
            if (token != NULL) {
                float new_value = atof(token);
                
                for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
                    if (strcmp(param_name, param_names[i]) == 0) {
                        if (validateParamRange(i, new_value)) {
                            *((float*)&current_params + i) = new_value;
                            saveParamsToFlash();
                            
                            Serial.print("Set ");
                            Serial.print(param_names[i]);
                            Serial.print(" to ");
                            Serial.println(new_value, 4);
                            
                            // 更新滤波器参数
                            if (strcmp(param_name, "IMU_FILTER") == 0) {
                                accel_x_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
                                accel_y_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
                                gyro_z_filter.setCutoffFrequency(2*current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
                            }
                            if (strcmp(param_name, "SERVO_FILTER") == 0) {
                                servo_filter.setCutoffFrequency(current_params.SERVO_FILTER, current_params.LOOP_FREQUENCY);
                            }
                            
                            Serial.println("Note: Some parameters require restart to take effect");
                        } else {
                            Serial.print("Value out of range! Valid range: ");
                            Serial.print(param_ranges[i].min);
                            Serial.print(" to ");
                            Serial.println(param_ranges[i].max);
                        }
                        return;
                    }
                }
                Serial.print("Unknown parameter: ");
                Serial.println(param_name);
            }
        }
    }
    else if (strcmp(token, "help") == 0) {
        Serial.println("Available commands:");
        Serial.println("  report on/off - Enable/disable data reporting");
        Serial.println("  get <param>   - Get parameter value");
        Serial.println("  set <param> <value> - Set parameter value");
        Serial.println("  help          - Show this help");
        Serial.println("  params        - List all parameters");
    }
    else if (strcmp(token, "params") == 0) {
        Serial.println("Available parameters:");
        for (uint8_t i = 0; i < sizeof(param_names)/sizeof(char*); i++) {
            float value = *((float*)&current_params + i);
            Serial.print("  ");
            Serial.print(param_names[i]);
            Serial.print(" = ");
            Serial.println(value, 4);
        }
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(token);
        Serial.println("Type 'help' for available commands");
    }
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
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
    Serial.begin(115200);
    delay(100);
    
    // 加载参数
    Serial.println("Loading parameters...");
    loadParamsFromFlash();

    // 初始化IMU
    Serial.println("Initializing IMU...");
    if (imu.begin() != 0) {
        Serial.println("ERROR: LSM6DS3 initialization failed!");
        while(1) {}
    } else {
        Serial.println("LSM6DS3 initialized successfully!");
    }
    
    // 初始化硬件PWM输出
    if (!initPWMOutput()) {
        Serial.println("ERROR: PWM output initialization failed!");
        while(1) {}
    }
    
    // 初始化PWM输入捕获
    if (!initPWMCapture()) {
        Serial.println("ERROR: PWM capture initialization failed!");
        // 注意：即使捕获失败，程序仍然可以运行，使用默认值
    }
    
    // 初始化滤波器
    accel_x_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    accel_y_filter.setCutoffFrequency(current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    gyro_z_filter.setCutoffFrequency(2*current_params.IMU_FILTER, current_params.LOOP_FREQUENCY);
    servo_filter.setCutoffFrequency(current_params.SERVO_FILTER, current_params.LOOP_FREQUENCY);
    
    // 初始化时间戳
    steering_last_capture_time = millis();
    gain_last_capture_time = millis();
    
    delay(100);
    
    Serial.println("nRF52840 RC gyro system");
    Serial.println("PWM capture: Optimized interrupt (CHANGE trigger)");
    Serial.print("Installation direction: ");
    Serial.print((int)current_params.BOARD_ROTATION);
    Serial.println(" degree");
    Serial.println("Serial commands: report on/off, get/set <param>, help");
    Serial.println("Data reporting is OFF by default");
}

void loop() {
    uint32_t loop_start_time = micros();
    
    // 处理串口输入
    processSerialInput();
    
    // 检查输入超时
    checkInputTimeout();
    
    // 读取传感器数据
    float accel_x_raw = imu.readFloatAccelX();
    float accel_y_raw = imu.readFloatAccelY();
    float gyro_z_raw = imu.readFloatGyroZ();

    gyro_z_raw = median_filter(gyro_z_raw);//去毛刺
    
    float accel_x_filt, accel_y_filt, gyro_z_filt;
    applySensorFilters(accel_x_raw, accel_y_raw, gyro_z_raw, 
                      accel_x_filt, accel_y_filt, gyro_z_filt);
    
    KinematicState state;
    calculateKinematicState(accel_x_filt, accel_y_filt, gyro_z_filt, state, current_params.BOARD_ROTATION);
    
    // 获取当前PWM值
    uint16_t steering_pwm = getSteeringPWM();
    float gain = calculateGain();
    float correction = calculateCounterSteerByKinematics(state, gain);
    
    // 输出到舵机
    outputServo(steering_pwm, correction);
    
    // 数据报告
    static uint32_t last_print = 0;
    if (millis() - last_print > 100) {
        printKinematicData(state, correction);
        last_print = millis();
    }
    
    // 控制循环频率
    controlLoopFrequency();
}