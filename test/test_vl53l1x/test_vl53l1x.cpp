#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "config.h"

#include <VL53L1X.h>

// Default to one-sensor mode; enable the end-gate path with -DENABLE_END_GATE_SENSOR=1.
#ifndef ENABLE_END_GATE_SENSOR
#define ENABLE_END_GATE_SENSOR 0
#endif

#if ENABLE_END_GATE_SENSOR
#define DUAL_SENSOR_TESTS 1
#else
#define DUAL_SENSOR_TESTS 0
#endif

#ifndef REQUIRE_END_GATE_SENSOR
#define REQUIRE_END_GATE_SENSOR 0
#endif

static VL53L1X start_gate_sensor;
#if DUAL_SENSOR_TESTS
static VL53L1X end_gate_sensor;
#endif
static bool tests_started = false;
static bool tests_finished = false;
static unsigned long boot_ms = 0;
static unsigned long last_heartbeat_ms = 0;
static bool start_sensor_ready = false;
#if DUAL_SENSOR_TESTS
static bool end_sensor_ready = false;
#endif

static bool i2c_device_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool recover_i2c_bus() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(1);

  if (digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH) {
    return true;
  }

  pinMode(SCL, OUTPUT);
  for (uint8_t i = 0; i < 16; ++i) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }

  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delay(1);
  return (digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH);
}

static void log_i2c_line_state(const char *tag) {
  char msg[64];
  snprintf(msg, sizeof(msg), "[TEST] i2c: %s SDA=%d SCL=%d",
           tag, digitalRead(SDA), digitalRead(SCL));
  TEST_MESSAGE(msg);
}

static uint8_t scan_i2c_bus() {
  uint8_t found_count = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      ++found_count;
      char msg[40];
      snprintf(msg, sizeof(msg), "[TEST] i2c: found device at 0x%02X", addr);
      TEST_MESSAGE(msg);
    }
  }
  return found_count;
}

static bool wait_for_i2c_device(uint8_t addr, uint16_t timeout_ms) {
  const unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (i2c_device_present(addr)) {
      return true;
    }
    delay(5);
  }
  return false;
}

static void reset_sensors_with_xshut() {
  pinMode(START_GATE_XSHUT_PIN, OUTPUT);
  digitalWrite(START_GATE_XSHUT_PIN, LOW);
#if DUAL_SENSOR_TESTS
  pinMode(END_GATE_XSHUT_PIN, OUTPUT);
  digitalWrite(END_GATE_XSHUT_PIN, LOW);
#endif
  delay(10);
}

static void wake_sensor(uint8_t xshut_pin) {
  // Match main.cpp behavior: release XSHUT so the board pull-up powers sensor.
  pinMode(xshut_pin, INPUT);
  delay(10);
}

static void configure_sensor(VL53L1X &sensor) {
  sensor.setTimeout(100);
  bool initialized = sensor.init();
  TEST_ASSERT_TRUE_MESSAGE(initialized, "VL53L1X init() failed");

  bool mode_ok = sensor.setDistanceMode(VL53L1X::Short);
  TEST_ASSERT_TRUE_MESSAGE(mode_ok, "setDistanceMode(Short) failed");

  bool budget_ok = sensor.setMeasurementTimingBudget(20000);
  TEST_ASSERT_TRUE_MESSAGE(budget_ok, "setMeasurementTimingBudget(20000) failed");
}

void test_i2c_initial_scan() {
  TEST_MESSAGE("[TEST] i2c: reset + wake start sensor for initial scan");
  reset_sensors_with_xshut();
  wake_sensor(START_GATE_XSHUT_PIN);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  log_i2c_line_state("before recover");

  if (!recover_i2c_bus()) {
    log_i2c_line_state("after recover");
    TEST_IGNORE_MESSAGE("I2C bus held low. Check sensor power, GND, SDA/SCL wiring, and pull-ups.");
  }

  log_i2c_line_state("after recover");
  TEST_ASSERT_TRUE_MESSAGE(wait_for_i2c_device(0x29, 700),
                           "Default VL53L1X address 0x29 did not respond before scan");

  TEST_MESSAGE("[TEST] i2c: running full bus scan");

  uint8_t found_count = scan_i2c_bus();
  char msg[48];
  snprintf(msg, sizeof(msg), "[TEST] i2c: total devices found = %u", found_count);
  TEST_MESSAGE(msg);

  TEST_ASSERT_TRUE_MESSAGE(i2c_device_present(0x29),
                           "No device responded at default VL53L1X address 0x29");
}

void test_vl53l1x_start_gate_init_and_address_change() {
  TEST_MESSAGE("[TEST] start_gate: reset + wake");
  reset_sensors_with_xshut();
  wake_sensor(START_GATE_XSHUT_PIN);

  if (!recover_i2c_bus()) {
    log_i2c_line_state("start_gate recover failed");
    TEST_IGNORE_MESSAGE("Skipping start gate test: I2C bus held low.");
  }

  TEST_ASSERT_TRUE_MESSAGE(wait_for_i2c_device(0x29, 500),
                           "Start gate did not appear at default address 0x29");

  TEST_MESSAGE("[TEST] start_gate: configure + set address");
  configure_sensor(start_gate_sensor);
  start_gate_sensor.setAddress(START_GATE_SENSOR_ADDR);

  TEST_ASSERT_TRUE_MESSAGE(i2c_device_present(START_GATE_SENSOR_ADDR),
                           "Start gate sensor not found at configured address");
  start_sensor_ready = true;
}

#if DUAL_SENSOR_TESTS
void test_vl53l1x_end_gate_init_and_address_change() {
  TEST_MESSAGE("[TEST] end_gate: wake second sensor");
  wake_sensor(END_GATE_XSHUT_PIN);

  if (!recover_i2c_bus()) {
    log_i2c_line_state("end_gate recover failed");
    TEST_IGNORE_MESSAGE("Skipping end gate test: I2C bus held low.");
  }

  if (!wait_for_i2c_device(0x29, 500)) {
    char pin_msg[64];
    snprintf(pin_msg, sizeof(pin_msg), "[TEST] end_gate: XSHUT pin state=%d", digitalRead(END_GATE_XSHUT_PIN));
    TEST_MESSAGE(pin_msg);

    uint8_t found_count = scan_i2c_bus();
    char msg[56];
    snprintf(msg, sizeof(msg), "[TEST] end_gate: scan found %u device(s)", found_count);
    TEST_MESSAGE(msg);

#if REQUIRE_END_GATE_SENSOR
    TEST_FAIL_MESSAGE("End gate did not appear at default address 0x29");
#else
    TEST_IGNORE_MESSAGE("End gate not detected at 0x29; skipping in non-strict mode.");
#endif
  }

  TEST_MESSAGE("[TEST] end_gate: configure + set address");
  configure_sensor(end_gate_sensor);
  end_gate_sensor.setAddress(END_GATE_SENSOR_ADDR);

  TEST_ASSERT_TRUE_MESSAGE(i2c_device_present(START_GATE_SENSOR_ADDR),
                           "Start gate sensor missing after end gate bring-up");
  TEST_ASSERT_TRUE_MESSAGE(i2c_device_present(END_GATE_SENSOR_ADDR),
                           "End gate sensor not found at configured address");
  end_sensor_ready = true;
}
#endif

void test_vl53l1x_continuous_ranging_produces_measurements() {
  TEST_MESSAGE("[TEST] ranging: start continuous mode");

  if (!start_sensor_ready) {
    TEST_IGNORE_MESSAGE("Skipping ranging test: start sensor not initialized.");
  }
#if DUAL_SENSOR_TESTS
  if (!end_sensor_ready) {
    TEST_IGNORE_MESSAGE("Skipping ranging test: end sensor not initialized.");
  }
#endif

  if (!recover_i2c_bus()) {
    log_i2c_line_state("ranging recover failed");
    TEST_IGNORE_MESSAGE("Skipping ranging test: I2C bus held low.");
  }

  start_gate_sensor.startContinuous(25);
#if DUAL_SENSOR_TESTS
  end_gate_sensor.startContinuous(25);
#endif
  delay(30);

  uint16_t start_range_mm = 0;
#if DUAL_SENSOR_TESTS
  uint16_t end_range_mm = 0;
#endif

  for (uint8_t i = 0; i < 20; ++i) {
    start_range_mm = start_gate_sensor.read(false);
#if DUAL_SENSOR_TESTS
    end_range_mm = end_gate_sensor.read(false);
    if ((start_range_mm > 0) && (end_range_mm > 0)) {
      break;
    }
#else
    if (start_range_mm > 0) {
      break;
    }
#endif
    delay(10);
  }

  TEST_ASSERT_FALSE_MESSAGE(start_gate_sensor.timeoutOccurred(),
                            "Start gate ranging timed out");
#if DUAL_SENSOR_TESTS
  TEST_ASSERT_FALSE_MESSAGE(end_gate_sensor.timeoutOccurred(),
                            "End gate ranging timed out");
#endif
  TEST_ASSERT_TRUE_MESSAGE(start_range_mm > 0, "Start gate returned 0 mm");
#if DUAL_SENSOR_TESTS
  TEST_ASSERT_TRUE_MESSAGE(end_range_mm > 0, "End gate returned 0 mm");
#endif

  start_gate_sensor.stopContinuous();
#if DUAL_SENSOR_TESTS
  end_gate_sensor.stopContinuous();
#endif
}

static void run_all_tests() {
  Serial.println("[TEST] Boot complete, preparing Unity run...");
  Serial.flush();
  delay(4000);
  Serial.println("[TEST] Starting Unity tests");
  Serial.flush();
  UNITY_BEGIN();
  Serial.println("[TEST] RUN_TEST i2c_initial_scan");
  Serial.flush();
  RUN_TEST(test_i2c_initial_scan);
  Serial.println("[TEST] RUN_TEST start_gate_init_and_address_change");
  Serial.flush();
  RUN_TEST(test_vl53l1x_start_gate_init_and_address_change);
#if DUAL_SENSOR_TESTS
  Serial.println("[TEST] RUN_TEST end_gate_init_and_address_change");
  Serial.flush();
  RUN_TEST(test_vl53l1x_end_gate_init_and_address_change);
#else
  Serial.println("[TEST] single-sensor mode: skipping end_gate test");
  Serial.flush();
#endif
  Serial.println("[TEST] RUN_TEST continuous_ranging_produces_measurements");
  Serial.flush();
  RUN_TEST(test_vl53l1x_continuous_ranging_produces_measurements);
  UNITY_END();
}

void setup() {
  Serial.begin(BRIDGE_BAUDRATE);
  delay(200);
  Serial.println("[TEST] setup() entered");
  Serial.flush();

  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif

  boot_ms = millis();
  last_heartbeat_ms = boot_ms;
  Serial.println("[TEST] waiting 5s before Unity start (monitor attach window)");
  Serial.flush();
}

void loop() {
  const unsigned long now = millis();

  if (!tests_started && (now - last_heartbeat_ms >= 1000UL)) {
    last_heartbeat_ms = now;
    Serial.println("[TEST] heartbeat: waiting to start");
    Serial.flush();
  }

  if (!tests_started && (now - boot_ms >= 5000UL)) {
    tests_started = true;
    run_all_tests();
    tests_finished = true;
    Serial.println("[TEST] Unity run finished");
    Serial.flush();
  }

  if (tests_finished && (now - last_heartbeat_ms >= 5000UL)) {
    last_heartbeat_ms = now;
    Serial.println("[TEST] heartbeat: idle after test run");
    Serial.flush();
  }
}