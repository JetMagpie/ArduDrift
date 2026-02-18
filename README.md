# ArduDrift : RC Drift Car Gyro Stabilization System

An Arduino-based gyro stabilization system for RC drift cars using seeed xiao ble sense hardware. This project implements counter-steering control using dual sensor parameters (angular velocity and lateral acceleration) to maintain drift angles and improve vehicle control.

## Features

- **Dual Sensor Fusion**: Combines gyroscope (angular velocity) and accelerometer (lateral acceleration) data for precise counter-steering control
- **Hardware PWM Output**: Uses hardware timers for stable 50Hz servo control
- **Configurable Installation**: Supports flight controller rotation
- **Real-time Adjustability**: Gain control via remote control channel
- **Sensor Filtering**: Multiple low-pass filters for smooth signal processing
- **Serial Command Interface**: Real-time parameter adjustment via serial connection
- **EEPROM Storage**: All parameters automatically saved and restored
- **S-Curve Response**: Configurable non-linear response curves for gyro output and servo response

## Hardware Requirements

- **Main Controller**: seeed xiao ble sense
- **Sensors**: Built-in IMU
- **Test Platform**: 3Racing D5MR Chassis (validated)
- **Power**: 5V BEC (external) - **Important: Do not use 6V ESC power**

![D5MR with xiao](./D5mr%20with%20xiao.jpg)
*3Racing D5MR drift chassis with xiao ble sense*

## Wiring Configuration

| Signal Type | Xiao ble Pin | Physical Connection |
|-------------|------------|---------------------|
| Steering Input | Pin D4 | Receiver steering channel |
| Gain Input | Pin D5 | Receiver auxiliary channel |
| Servo Output | Pin D1 | Steering servo signal |

## Power Supply Warning

**CRITICAL**: xiao ble cannot typically handle 6V power from car ESCs. 

**Required Setup**:
1. Connect an external 5V BEC to the Vbus and GND pins
2. Ensure all components share a common ground

## Installation & Programming

1. **Software Setup**:
   - Install Arduino IDE
   - Add xiao ble board support

 Navigate to File > Preferences, and fill "Additional Boards Manager URLs" with the url below: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

 Navigate to Tools > Board > Boards Manager..., type the keyword "seeed nrf52" in the search box, select the latest version of the board you want, and install it. 

 Download Seeed_Arduino_LSM6DS3 Library as a zip file: https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

 Open Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library... and open the downloaded zip file.

2. **Upload Code**:
   - Open the project in Arduino IDE
   - Select appropriate board and port
   - Upload to xiao ble sense

3. **Hardware Installation**:
   - Mount xiao ble sense in desired orientation
   - Update `BOARD_ROTATION` in code if rotated (default: 90°)
   - Connect all signals as per wiring table

## Serial Command Interface

The system provides a comprehensive serial command interface for real-time parameter adjustment. Connect to the xiao ble sense via USB at 115200 baud.

### Available Commands:

| Command | Description | Example |
|---------|-------------|---------|
| `report on` | Enable real-time data reporting | `report on` |
| `report off` | Disable real-time data reporting | `report off` |
| `get <param>` | Get current parameter value | `get K_GAIN` |
| `set <param> <value>` | Set parameter value | `set K_GAIN 0.02` |
| `params` | List all parameters and current values | `params` |
| `help` | Show command help | `help` |

### Usage Examples:

```bash
# Enable data monitoring
report on

# Check current gain value
get K_GAIN

# Increase gain sensitivity
set K_GAIN 0.02

# List all parameters
params

# Disable data output when tuning
report off
```
## Parameter Tables

### System Parameters

| Parameter | Default Value | Valid Range | Description |
|-----------|---------------|-------------|-------------|
| `BOARD_ROTATION` | 90 | 0-360° | Controller installation angle |
| `K_GAIN` | 0.003 | -0.1-0.1 | Overall sensitivity multiplier, use a negative value to reverse servo rotation |
| `DEFAULT_GAIN` | 200 | 0-500 | Default gain when no input signal |

### Control Parameters

| Parameter | Default Value | Valid Range | Description |
|-----------|---------------|-------------|-------------|
| `STEER_BY_ACC_RATE` | 0.5 | 0-20 | Reducing angle integration by lateral acceleration ratio |
| `COUNTER_STEER_RANGE` | 0.95 | 0-1.0 | Maximum counter-steering output range |
| `STEER_BY_ANGACC_RATE` | 1 | 0-10 | Angular acceleration contribution ratio |
| `STEER_BY_ANGVEL_RATE` | 1.1 | 0-20 | Angular velocity contribution ratio |
| `STEER_BY_ANG_RATE` | 1 | 0-20 | Angle integration contribution ratio |
| `STEER_BY_ANG_LIMIT` | 90 | 10-90 | Maximum angle integration limit (degrees) |
| `ANGVEL_ZERO` | 0 | -20-20 | Gyroscope zero offset calibration |
| `GYRO_EXP` | -0.18 | -1 to 1 | Gyro output S-curve exponent (0=linear) |
| `OUTPUT_EXP` | 0 | -1 to 1 | Servo output S-curve exponent (0=linear) |
| `ANG_HALF_LIFE` | 0.15 | 0.001 to 2 | Angle integration reducing time |

### Servo Parameters

| Parameter | Default Value | Valid Range | Description |
|-----------|---------------|-------------|-------------|
| `SERVO_LIMIT_LEFT` | 1.0 | 0-1.0 | Left servo limit (1.0 = full range) |
| `SERVO_LIMIT_RIGHT` | 1.0 | 0-1.0 | Right servo limit (1.0 = full range) |

### Filter Parameters

| Parameter | Default Value | Valid Range | Description |
|-----------|---------------|-------------|-------------|
| `LOOP_FREQUENCY` | 100 | 50-1000Hz | Main control loop frequency |
| `IMU_FILTER` | 30 | 1-500Hz | IMU sensor filter cutoff frequency |
| `SERVO_FILTER` | 120 | 1-500Hz | Servo output filter for smoothing |
| `ANGACC_FILTER` | 30 | 1-500Hz | Angular acceleration filter |

## Control Algorithm

The system uses a simplified physics-based approach:

### Counter-Steering Components:

1. **Angular Velocity Component**:
   - Right rotation (negative angular_vel) → Left counter-steer (negative output)
   - Left rotation (positive angular_vel) → Right counter-steer (positive output)
   - Angular acceleration is used to improve response speed

2. **Angle Integration Component**:
   - Integrates angular velocity over time to track drift angle
   - Provides sustained counter-steering
   - Prevents integral windup with configurable saturation limits

3. **Angular Integration Reduction**:
   - Exponential decay with a half life variable
   - Linear decay with lateral acceleration

4. **S-Curve Response**:
   - **Gyro Output S-Curve** (`GYRO_EXP`): Adjusts sensitivity distribution in counter-steering calculation
     - Positive values: More sensitive in mid-range, less sensitive at extremes
     - Negative values: Less sensitive in mid-range, more sensitive at extremes
   
   - **Servo Output S-Curve** (`OUTPUT_EXP`): Adjusts sensitivity distribution in final servo output
     - Same exponential behavior applied to combined steering and correction signals
     - Allows fine-tuning of steering feel and response characteristics

5. **Combined Output**:
   - Both components are summed and scaled by user gain
   - Final output limited to ±1.0 for servo safety

## Performance & Validation

- **Successfully Tested**: Maintains consistent drift circles and transition maneuvers
- **Vehicle Platform**: 3Racing D5MR chassis
- **Control Stability**: Smooth counter-steering response without oscillation

## Parameter Tuning

Current default parameters work well for medium-speed drifting. Parameters are continuously updated based on track testing. Future versions will provide better presets for different driving styles and track conditions.

## Development Notes

- **AI Assistance**: Code development supported by Deepseek AI
- **Future Support**: Planned compatibility with additional development boards
- **Open Source**: Contributions and improvements welcome

## Troubleshooting

1. **No Servo Movement**: Check servo connection and 5V BEC power
2. **No Receiver Input**: Verify receiver connections and receiver binding
3. **Erratic Behavior**: Ensure stable 5V power supply and proper grounding
4. **Wrong Direction**: Adjust `BOARD_ROTATION` and 'K_GAIN' parameter
5. **Serial Connection Issues**: Verify 115200 baud rate and correct COM port

## Contributing

We believe in the power of open collaboration! Contributions are what make the open-source community such an amazing place to learn, inspire, and create.

## License

Distributed under the **GNU General Public License v3.0**. 

This means:
- You can use this software for any purpose
- You can study how it works and change it
- You can distribute copies to help others
- You can distribute your modified versions

**The only requirement:** If you distribute modified versions, you must release your changes under the same GPL v3 license.

See the [LICENSE](LICENSE) file for details, or visit:
https://www.gnu.org/licenses/gpl-3.0.html

---

**Happy Drifting!**
