# Arduino Drift System Tuner

A comprehensive tuning and monitoring application for ardudrift control system.

## Features

- **Real-time Parameter Tuning**: Adjust system parameters via intuitive sliders
- **Serial Communication**: Connect to Arduino devices via serial port
- **Live Data Monitoring**: Real-time sensor data visualization with multiple plots
- **S-Curve Visualization**: Preview response curves for gyro and output parameters
- **Flash Management**: Save and load parameters to/from device flash
- **Manual Command Interface**: Send custom commands directly to the device

## Supported Parameters

### System Parameters
- **BOARD_ROTATION**: Controller installation orientation (0-360°)
- **K_GAIN**: Overall sensitivity multiplier, use a negative value to reverse servo rotation
- **DEFAULT_GAIN**: Default gain when no input signal
- **LOOP_FREQUENCY**: Main control loop frequency (50-1000Hz)

### Control Parameters
- **STEER_BY_ACC_RATE**: Reducing angle integration by lateral acceleration ratio
- **STEER_BY_ANGVEL_RATE**: Angular velocity contribution ratio
- **STEER_BY_ANG_RATE**: Angle integration contribution ratio
- **STEER_BY_ANGACC_RATE**: Angular acceleration contribution ratio
- **STEER_BY_ANG_LIMIT**: Maximum angle integration limit
- **COUNTER_STEER_RANGE**: Maximum counter-steering output range
- **ANGVEL_ZERO**: Gyroscope zero offset calibration
- **ANG_HALF_LIFE**: Angle integration reducing time

### Servo Parameters
- **SERVO_LIMIT_LEFT**: Left servo limit (0.0-1.0)
- **SERVO_LIMIT_RIGHT**: Right servo limit (0.0-1.0)

### Filter Parameters
- **IMU_FILTER**: IMU sensor filter cutoff frequency
- **SERVO_FILTER**: Servo output filter for smoothing
- **ANGACC_FILTER**: Angular acceleration filter

### S-Curve Parameters
- **GYRO_EXP**: Gyro output S-curve exponent (-1 to 1)
- **OUTPUT_EXP**: Servo output S-curve exponent (-1 to 1)


## Data Monitoring

The application can display real-time sensor data including:
- PWM inputs (steering, gain)
- Servo output
- Acceleration data (X, Y axes)
- Angular velocity
- Correction values

## Requirements

- Python 3.7+
- PyQt5
- pyserial
- matplotlib
- numpy

## Usage

1. Connect your Arduino device
2. Select the correct serial port
3. Click "Open Port" to establish connection
4. Use sliders to adjust parameters in real-time
5. Enable "Report" to start data monitoring
6. Open "Real-time Charts" for visual data analysis