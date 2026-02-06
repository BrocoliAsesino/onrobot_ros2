#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>

#include "onrobot_driver/IModbusConnection.hpp"
#include "onrobot_driver/TCPConnectionWrapper.hpp"
#include "onrobot_driver/SerialConnectionWrapper.hpp"

// Modbus request/response definitions and utilities
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"


class VG {
public:
    // Constructors for TCP and Serial connections
    VG(const std::string &type, const std::string &ip, int port);
    VG(const std::string &type, const std::string &device);
    ~VG();

    // Vacuum control commands
    bool setVacuumLevel(float vacuum_val);
    bool activateGripper();
    bool releaseGripper();

    // Status and utility methods
    float getVacuumLevel(uint16_t CHANNEL);
    std::vector<int> getStatus();
    std::vector<float> getStatusAndPrint();

    // Constants for registers and modes
    static constexpr uint16_t DEVICE_ID = 65;
    static constexpr uint16_t VG_REG_CTRL_A = 0;
    static constexpr uint16_t VG_REG_CTRL_B = 1;
    static constexpr uint16_t STATUS_ADDR_A_VACUUM = 18;
    static constexpr uint16_t STATUS_ADDR_B_VACUUM = 19;
    static constexpr uint16_t PUMP_SPEED = 24;

    static constexpr uint16_t VG_STATUS_START = 18;
    static constexpr uint16_t VG_STATUS_COUNT = 7;

    // Modes
    static constexpr uint8_t MODE_RELEASE = 0x00;
    static constexpr uint8_t MODE_GRIP    = 0x01;
    static constexpr uint8_t MODE_IDLE    = 0x02;

private:
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    // Helper methods
    uint16_t buildCommandWord(uint8_t mode, uint8_t vacuum);
    bool setTargetVacuumLevel(uint8_t MODE, float vacuum_val);
    bool writeRegister(uint16_t address, uint16_t value);
    bool writeMultipleRegisters(uint16_t address, const std::vector<uint16_t>& values);

    // Connection and configuration
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    float max_vacuum;
    float default_vacuum;
};