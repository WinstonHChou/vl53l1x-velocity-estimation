#pragma once

#define DEBUG_SERIAL

// 1. Hardware Configuration
#define START_GATE_GPIO1_PIN    2
#define END_GATE_GPIO1_PIN      3

#define START_GATE_XSHUT_PIN    4
#define END_GATE_XSHUT_PIN      5

#define START_GATE_SENSOR_ADDR  0x30
#define END_GATE_SENSOR_ADDR    0x31

// 2. Communication Parameters
#define I2C_MIN_ADDRESS 0x00
#define I2C_MAX_ADDRESS 0x7F
#define I2C_STANDARD_MODE_CLOCK_FREQ 100000L
#define I2C_FAST_MODE_CLOCK_FREQ 400000L

#define I2C_CLOCK_FREQ I2C_FAST_MODE_CLOCK_FREQ
#define BRIDGE_BAUDRATE 115200  // need headroom for 13+ sensor packets per cycle (was 460800; 13th sensor still dropped)

#define UPDATE_INTERVAL_MS      100

// 3. Physics & Calibration
#define BASELINE_MM 87.50f
#define TRIGGER_THRESHOLD_MM 400
