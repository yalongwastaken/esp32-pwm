# ESP32 PWM Control
A collection of ESP32 PWM (Pulse Width Modulation) implementations showcasing progression from basic LED control to advanced multi-channel RGB systems. These examples demonstrate various PWM techniques including direct control, potentiometer-based adjustment, and synchronized multi-channel management with robust error handling.

## Project Structure
### Arduino Framework (`arduino/`)
* `pwm_basic/` - Fundamental PWM control for single LED brightness adjustment
* `pwm_advanced/` - Potentiometer-controlled PWM with ADC integration
* `pwm_multi-channel/` - RGB LED control system with three synchronized PWM channels

### ESP-IDF Framework (`esp-idf/`)
* `pwm_basic/` - LEDC peripheral implementation for precise PWM generation
* `pwm_advanced/` - Advanced PWM control with ADC sampling and input validation
* `pwm_multi-channel/` - Production-ready RGB controller with FreeRTOS task management and error queuing

## Development Environment
**Primary Development:** PlatformIO IDE

**Compatibility:**
* **Arduino projects:** Compatible with Arduino IDE
* **ESP-IDF projects:** Compatible with native `idf.py` toolchain

**Hardware Tested:** ESP32-WROOM-32E module

## Potential Improvements & Add-ons
* Gamma correction for linear brightness perception
* Color temperature presets and transitions
* WS2812B/addressable LED strip support
* PWM frequency optimization for different LED types
* Web interface for remote color control
* BLE/WiFi connectivity for smartphone integration
* Power consumption monitoring and optimization
* HSV color space conversion
* Sunrise/sunset simulation modes
* Music-reactive lighting effects

## Author
Anthony Yalong