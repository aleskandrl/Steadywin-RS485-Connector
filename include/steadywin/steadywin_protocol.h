#pragma once

#include "steadywin/motor_error.h"
#include "steadywin/serial_port_interface.h"
#include "steadywin/steadywin_types.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace steadywin {

/**
 * @class SteadywinProtocol
 * @brief Implements the low-level RS485 communication protocol.
 * This class is responsible for building command packets, sending them,
 * and parsing response packets. It implements the "micro-methods" that
 * correspond directly to the protocol commands.
 */
class SteadywinProtocol {
public:
    /**
     * @brief Constructor.
     * @param port A shared pointer to an ISerialPort implementation.
     */
    explicit SteadywinProtocol(std::shared_ptr<ISerialPort> port);

    // --- System Commands ---

    /**
     * @brief [0x0B] Reads real-time data (angle, velocity, current, etc.).
     * @param device_address The address of the slave device.
     * @param[out] data A struct to be filled with the raw telemetry payload.
     * @return MotorError::Ok on success.
     */
    MotorError readRealtimeData(uint8_t device_address, RealtimeDataPayload& data);

    // --- Control Commands ---

    /**
     * @brief [0x22] Sets absolute position control mode.
     * The device will reply with a standard telemetry packet.
     * @param device_address The address of the slave device.
     * @param absolute_position_counts Target position in encoder counts (1 rotation = 16384).
     * @param[out] response_data The telemetry data received in the response.
     * @return MotorError::Ok on success.
     */
    MotorError setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data);

    /**
     * @brief [0x2F] Disables the motor output (freewheel).
     * The device will reply with a standard telemetry packet.
     * @param device_address The address of the slave device.
     * @param[out] response_data The telemetry data received in the response.
     * @return MotorError::Ok on success.
     */
    MotorError disableMotor(uint8_t device_address, RealtimeDataPayload& response_data);

    /**
     * @brief [0x0F] Clears faults.
     */
    MotorError clearFaults(uint8_t device_address, uint8_t& current_faults);

    /**
     * @brief [0x1D] Sets current position as zero point.
     */
    MotorError setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset);

    /**
     * @brief [0x21] Velocity control.
     */
    MotorError setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data);

    /**
     * @brief [0x23] Relative position control.
     */
    MotorError setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data);

    /**
     * @brief [0x2E] Holding brake control.
     * @param operation 0x00: Open, 0x01: Closed, 0xFF: Read
     */
    MotorError setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status);

    /**
     * @brief [0x14] Read motion control parameters.
     */
    MotorError readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params);

    /**
     * @brief [0x15] Write motion control parameters.
     */
    MotorError writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params);

private:
    /**
     * @brief The core function to send a command and wait for a response.
     * @param device_address The target device address.
     * @param command_code The command code byte.
     * @param request_payload The data payload to send with the command.
     * @param[out] response_payload The data payload from the device's response.
     * @return MotorError::Ok on success.
     */
    MotorError sendAndReceive(uint8_t device_address, uint8_t command_code, const std::vector<uint8_t>& request_payload, std::vector<uint8_t>& response_payload);

    /**
     * @brief Calculates the CRC16-MODBUS checksum for a given data buffer.
     * @param data Pointer to the data buffer.
     * @param length The number of bytes in the buffer.
     * @return The 2-byte CRC value.
     */
    static uint16_t calculateCrc16Modbus(const uint8_t* data, size_t length);

    std::shared_ptr<ISerialPort> port_;
    uint8_t packet_sequence_{0}; // Internal counter for packet sequence number
    
    // Default timeout for serial communication in milliseconds
    static constexpr unsigned int DEFAULT_TIMEOUT_MS = 100;
};

} // namespace steadywin
