# Arduino Drift System Tuner

A comprehensive tuning and monitoring application for ardudrift control system.

## Features

- **Real-time Parameter Tuning**: Adjust system parameters via intuitive sliders
- **Serial Communication**: Connect to Arduino devices via serial port
- **Live Data Monitoring**: Real-time sensor data visualization with multiple plots
- **S-Curve Visualization**: Preview response curves for gyro and output parameters
- **EEPROM Management**: Save and load parameters to/from device EEPROM
- **Manual Command Interface**: Send custom commands directly to the device

## Supported Parameters

- Board rotation and installation orientation
- Gain settings (K_GAIN, DEFAULT_GAIN)
- Steering response rates (STEER_BY_ACC_RATE, STEER_BY_ANGACC_RATE)
- Servo limits and ranges
- Filter settings (IMU, SERVO, ANGACC filters)
- Loop frequency control
- S-curve exponents (GYRO_EXP, OUTPUT_EXP)

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