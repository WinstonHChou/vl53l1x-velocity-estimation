#include "config.h"
#include "serial_bridge.hpp"

#include <Wire.h>
#include <VL53L1X.h>

SerialBridge bridge = SerialBridge();

VL53L1X startGateSensor, endGateSensor;

volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile bool shotComplete = false;

// --- Interrupt Service Routines ---
// Use IRAM_ATTR if you ever port this to ESP32, but for Uno standard is fine
void isrStart() {
    if (startTime == 0) { // Only catch the first edge
        startTime = micros(); 
    }
}

void isrEnd() {
    if (endTime == 0 && startTime != 0) {
        endTime = micros();
        shotComplete = true;
    }
}

void setup() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQ);
  Serial.begin(BRIDGE_BAUDRATE);
  bridge.begin(Serial);

  Serial.println("Program started. Initializing sensors...");
  Serial.println("--------------------------------");

  // 1. Disable/reset all sensors by driving their XSHUT pins low.
  pinMode(START_GATE_XSHUT_PIN, OUTPUT);
  digitalWrite(START_GATE_XSHUT_PIN, LOW);
  pinMode(END_GATE_XSHUT_PIN, OUTPUT);
  digitalWrite(END_GATE_XSHUT_PIN, LOW);
  delay(10);

  // 2. Set the XSHUT pins to INPUT to allow the carrier board's pull-up resistors to bring them high, powering on the sensors.
  pinMode(START_GATE_XSHUT_PIN, INPUT);
  delay(10);
  startGateSensor.init();
  startGateSensor.setAddress(START_GATE_SENSOR_ADDR);
  startGateSensor.setDistanceMode(VL53L1X::Short);
  startGateSensor.setMeasurementTimingBudget(20000); // 20ms is fastest for VL53L1X

  pinMode(END_GATE_XSHUT_PIN, INPUT);
  delay(10);
  endGateSensor.init();
  endGateSensor.setAddress(END_GATE_SENSOR_ADDR);
  endGateSensor.setDistanceMode(VL53L1X::Short);
  endGateSensor.setMeasurementTimingBudget(20000);  // 20ms is fastest for VL53L1X

  // Trigger interrupt only if distance < TRIGGER_THRESHOLD_MM mm (adjust for your gate width)
  startGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CONFIG_GPIO, 0x01); 
  startGateSensor.writeReg16Bit(VL53L1X::SYSTEM__THRESH_LOW, TRIGGER_THRESHOLD_MM); 

  endGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CONFIG_GPIO, 0x01);
  endGateSensor.writeReg16Bit(VL53L1X::SYSTEM__THRESH_LOW, TRIGGER_THRESHOLD_MM);

  // 3. Attach Interrupts
  pinMode(START_GATE_GPIO1_PIN, INPUT); // Pololu board provides the pull-up
  pinMode(END_GATE_GPIO1_PIN, INPUT);

  int startInterrupt = digitalPinToInterrupt(START_GATE_GPIO1_PIN);
  int endInterrupt = digitalPinToInterrupt(END_GATE_GPIO1_PIN);
  if (startInterrupt == NOT_AN_INTERRUPT || endInterrupt == NOT_AN_INTERRUPT) {
    Serial.println("ERROR: GPIO1 pins must map to external interrupts on this board.");
    while (1) {
      delay(1000);
    }
  }

  attachInterrupt(startInterrupt, isrStart, FALLING);
  attachInterrupt(endInterrupt, isrEnd, FALLING);

  // 4. Start Continuous Ranging
  startGateSensor.startContinuous(20);
  endGateSensor.startContinuous(20);

  Serial.println("\nStarting live readings...\n");
}

unsigned long lastMillis = 0;
void loop() {
  // Read sensors at defined sampling rate
  if (millis() - lastMillis >= UPDATE_INTERVAL_MS) {
    lastMillis = millis();

    // Snapshot shared ISR state atomically before computing with it.
    unsigned long shotStartUs;
    unsigned long shotEndUs;
    bool shotReady;
    noInterrupts();
    shotStartUs = startTime;
    shotEndUs = endTime;
    shotReady = shotComplete;
    interrupts();

    if (shotReady) {
      if (shotEndUs <= shotStartUs) {
        noInterrupts();
        startTime = 0;
        endTime = 0;
        shotComplete = false;
        interrupts();

        startGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        endGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        return;
      }

      // Calculate Delta T in seconds
      unsigned long tofUs = shotEndUs - shotStartUs;
      float dt = tofUs / 1000000.0f;
      float velocity = (BASELINE_MM / 1000.0f) / dt;

      // Pack and Send
      SensorPacket pkt = {};
      pkt.timestamp_ms = shotStartUs / 1000UL;
      pkt.time_of_flight_ms = tofUs / 1000UL;
      pkt.velocity_meters_per_second = velocity;
      bridge.send(pkt);

      // Reset for next fuel piece
      delay(500); // Debounce physical jitter/fuel shape
      noInterrupts(); // Protection while resetting volatiles
      startTime = 0;
      endTime = 0;
      shotComplete = false;
      interrupts();

      // IMPORTANT: Clear sensor interrupts to reset the physical GPIO pins
      startGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      endGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
    }

    if (!shotReady && shotStartUs != 0 && shotEndUs == 0 && (millis() - (shotStartUs / 1000UL) > 2000UL)) { // 2 second timeout
      #ifdef DEBUG_SERIAL
      Serial.println("Shot timed out - Resetting gate.");
      #endif
      noInterrupts();
      startTime = 0;
      endTime = 0;
      shotComplete = false;
      interrupts();
      startGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      endGateSensor.writeReg(VL53L1X::SYSTEM__INTERRUPT_CLEAR, 0x01);
    }

    unsigned long loop_time = millis() - lastMillis;
    if (loop_time > UPDATE_INTERVAL_MS) {
      #ifdef DEBUG_SERIAL
      Serial.println();
      Serial.print("Warning: Loop time ");
      Serial.print(loop_time);
      Serial.println("ms exceeds sampling interval!");
      #endif
      WatchdogPacket pkt = {};
      pkt.overrun = 1;
      pkt.loop_time_ms = loop_time;
      bridge.send(pkt);
    }
  }
}
