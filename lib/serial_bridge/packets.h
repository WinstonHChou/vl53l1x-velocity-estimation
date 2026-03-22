/**
 * @file packets.h
 * @brief Packet definitions for serial communication between microcontroller and host computer
 * 
 * Defines the packet structures used for serial protocol communication
 * between the host computer and microcontroller for sensor data acquisition and control.
 */
#pragma once
#include <stdint.h>


/**
 * @enum PacketID
 * @brief Identifier for packet types in serial communication protocol
 * 
 * Used to distinguish between different packet types transmitted over the serial link.
 * This identifier allows the receiver to correctly interpret the packet payload.
 * 
 * @var SENSOR
 *      Sensor data packet (value = 1). Sent from microcontroller to host
 *      containing measurements.
 */
enum PacketID : uint8_t {
    SENSOR  = 0x01,
    WATCHDOG = 0x02,
};


/**
 * @struct SensorPacket
 * @brief Sensor data packet sent from microcontroller to host
 * 
 * Contains real-time sensor measurements including pressure and force data.
 * 
 * @var timestamp_ms
 *      Timestamp in milliseconds since microcontroller start
 * @var time_of_flight_ms
 *      Time of flight measurement in milliseconds
 * @var velocity_meters_per_second
 *      Velocity measurement in meters per second
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;              // timestamp in milliseconds since microcontroller start
    uint32_t time_of_flight_ms;         // time of flight measurement in milliseconds
    float velocity_meters_per_second;   // velocity measurement in meters per second
} SensorPacket;

/** @struct WatchdogPacket
 * @brief Watchdog/status packet sent from microcontroller to host
 * 
 * Contains system status information such as loop timing and overrun.
 * 
 * @var overrun
 *      Flag indicating if an overrun occurred (1 = overrun, 0 = no overrun)
 * @var loop_time_ms
 *      Time taken for the main loop in milliseconds
 */
typedef struct __attribute__((packed)) {
    uint8_t overrun;
    uint32_t loop_time_ms;
} WatchdogPacket;
