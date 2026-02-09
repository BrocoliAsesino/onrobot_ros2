#include "onrobot_drivers/vg/VG.hpp"

VG::VG(const std::string &type, const std::string &ip, int port)
    : type(type)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "vgc10" && type != "vg10")
        throw std::invalid_argument("Please specify either 'vgc10' or 'vg10'.");

    // Attempt to establish TCP connection, retrying until successful
    while (true) {
        try {
            connection = std::make_unique<TCPConnectionWrapper>(ip, port);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to establish TCP connection: " << ex.what()
                      << ". Retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    // TODO: Set max_width and max_force according to VG model
    if (type == "vgc10")
    {
        max_vacuum = 80.0f; 
    }
    else if (type == "vg10")
    {
        max_vacuum = 80.0f; 
    }
    default_vacuum = 0.0f;
    // Set these defaults on the gripper.
    setVacuumLevel(default_vacuum);
}

VG::VG(const std::string &type, const std::string &device)
    : type(type)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "vgc10" && type != "vg10")
        throw std::invalid_argument("Please specify either 'vgc10' or 'vg10'.");

    // Attempt to establish Serial connection, retrying until successful
    while (true) {
        try {
            connection = std::make_unique<SerialConnectionWrapper>(device);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to establish Serial connection: " << ex.what()
                      << ". Retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    if (type == "vgc10")
    {
        max_vacuum = 80.0f; 
    }
    else if (type == "vg10")
    {
        max_vacuum = 80.0f; 
    }
    default_vacuum = 0.0f;

    // Set these defaults on the gripper.
    setVacuumLevel(default_vacuum);
}

VG::~VG()
{
    if (connection)
        connection->close();
}

MB::ModbusResponse VG::sendRequest(const MB::ModbusRequest &req)
{
    try
    {
        return connection->sendRequest(req);
    }
    catch (const MB::ModbusException &ex)
    {
        std::cerr << "Modbus exception: " << ex.what() << std::endl;
        throw;
    }
}

// TODO: Implement VG-specific methods below
// ####################################################################################

bool VG::setVacuumLevel(float vacuum_val)
{
    if (vacuum_val <= 0.0f)
    {
        return setTargetVacuumLevel(MODE_RELEASE, 0.0f);
    }
    if (vacuum_val > max_vacuum)
        vacuum_val = max_vacuum;
    return setTargetVacuumLevel(MODE_GRIP, vacuum_val);
}

bool VG::activateGripper()
{
    std::cout << "Activating max suction in both channels: " << max_vacuum << std::endl;
    return setVacuumLevel(max_vacuum);
}

bool VG::releaseGripper()
{
    std::cout << "Deactivating gripper" << std::endl;
    return setVacuumLevel(0.0f);
}


// Helper to build command word from mode and vacuum level.
uint16_t VG::buildCommandWord(uint8_t mode, uint8_t vacuum)
{
    if (mode == MODE_GRIP && vacuum > 80)
        vacuum = 80;

    return (uint16_t(mode) << 8) | uint16_t(vacuum);
}


bool VG::setTargetVacuumLevel(uint8_t MODE, float vacuum_val)
{
    uint16_t address = VG_REG_CTRL_A; 
    uint16_t command_word = buildCommandWord(MODE, static_cast<uint8_t>(vacuum_val));

    std::vector<uint16_t> values = {command_word, command_word};
    
    return writeMultipleRegisters(address, values);
}

bool VG::writeRegister(uint16_t address, uint16_t value)
{   
    std::vector<MB::ModbusCell> values = {MB::ModbusCell((uint16_t)(value))};
    MB::ModbusRequest req(DEVICE_ID, MB::utils::WriteSingleAnalogOutputRegister, address, 1, values);
    try
    {
        sendRequest(req);
        return true;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Write failed at address " << address << std::endl;
        return false;
    }
}

bool VG::writeMultipleRegisters(uint16_t address, const std::vector<uint16_t>& values)
{
    std::vector<MB::ModbusCell> cells;
    cells.reserve(values.size());

    for (uint16_t v : values)
        cells.emplace_back(v);

    MB::ModbusRequest req(
        DEVICE_ID,
        MB::utils::WriteMultipleAnalogOutputHoldingRegisters,
        address,
        cells.size(),
        cells
    );
    try
    {
        sendRequest(req);
        return true;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "WriteMultiple failed at address " << address << std::endl;
        return false;
    }
}

float VG::getVacuumLevel(uint16_t CHANNEL)
{
    // Read Channel A vacuum: register 18 (permille)
    MB::ModbusRequest req(
        DEVICE_ID,
        MB::utils::ReadAnalogOutputHoldingRegisters,
        CHANNEL,  
        1
    );

    try
    {
        MB::ModbusResponse resp = sendRequest(req);

        uint16_t regValue = resp.registerValues().front().reg(); // 0–1000 permille

        // Convert permille → percent
        return float(regValue) / 10.0f;  // 523 → 52.3%
    }
    catch (const MB::ModbusException&)
    {
        std::cerr << "Failed to read actual vacuum level!" << std::endl;
        return -1.0f;
    }
}

std::vector<int> VG::getStatus()
{
    MB::ModbusRequest req(
        DEVICE_ID,
        MB::utils::ReadAnalogOutputHoldingRegisters,
        18,             // start address
        7               // registers: 18 -> 24
    );

    try
    {
        MB::ModbusResponse resp = sendRequest(req);

        std::vector<int> out;
        out.reserve(7);

        for (auto &cell : resp.registerValues())
            out.push_back(int(cell.reg()));

        return out;
    }
    catch (const MB::ModbusException&)
    {
        std::cerr << "Failed to read VG10 status registers!" << std::endl;
        return {};
    }
}

std::vector<float> VG::getStatusAndPrint()
{
    // Read registers 18–24 (total 7 registers)
    MB::ModbusRequest req(
        DEVICE_ID,
        MB::utils::ReadAnalogOutputHoldingRegisters,
        18,
        7
    );

    try
    {
        MB::ModbusResponse resp = sendRequest(req);

        const auto &cells = resp.registerValues();
        if (cells.size() < 7)
        {
            std::cerr << "VG10 returned too few status registers!" << std::endl;
            return {};
        }

        // Extract raw values
        uint16_t vacuumA_permille = cells[0].reg();
        uint16_t vacuumB_permille = cells[1].reg();
        uint16_t supplyCurrent_mA = cells[2].reg();
        uint16_t supplyVoltage_mV = cells[3].reg();
        uint16_t internal5V_mV    = cells[4].reg();
        uint16_t temp_centiDeg    = cells[5].reg();
        uint16_t pumpRPM          = cells[6].reg();

        // Convert to human-readable values
        float vacuumA_percent = vacuumA_permille / 10.0f;
        float vacuumB_percent = vacuumB_permille / 10.0f;
        float supplyVoltage_V = supplyVoltage_mV / 1000.0f;
        float internal5V_V    = internal5V_mV / 1000.0f;
        float temperature_C   = temp_centiDeg / 100.0f;

        std::cout << "=== VG10 Status ===" << std::endl;
        std::cout << "Vacuum A: " << vacuumA_percent << " %" << std::endl;
        std::cout << "Vacuum B: " << vacuumB_percent << " %" << std::endl;
        std::cout << "Supply Current: " << supplyCurrent_mA << " mA" << std::endl;
        std::cout << "Supply Voltage: " << supplyVoltage_V << " V" << std::endl;
        std::cout << "Internal 5V Rail: " << internal5V_V << " V" << std::endl;
        std::cout << "Temperature: " << temperature_C << " °C" << std::endl;
        std::cout << "Pump Speed: " << pumpRPM << " RPM" << std::endl;

        return {
            vacuumA_percent,
            vacuumB_percent,
            (float)supplyCurrent_mA,
            supplyVoltage_V,
            internal5V_V,
            temperature_C,
            (float)pumpRPM
        };
    }
    catch (const MB::ModbusException&)
    {
        std::cerr << "Failed to read VG10 status registers!" << std::endl;
        return {};  // empty vector = failure
    }
}