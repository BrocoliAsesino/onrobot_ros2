import time
from vgc10driver.modbus_tcp_client import ModbusTCPClient

# --- VG10 Register and Command Definitions ---
# Control Register Addresses (Holding Registers, FC 0x06)
VG10_CHANNEL_A_CTRL_ADDR = 0
VG10_CHANNEL_B_CTRL_ADDR = 1

# Control Mode Values (High Byte of Control Register) 
MODE_RELEASE = 0x00
MODE_GRIP = 0x01
MODE_IDLE = 0x02

# Status Register Addresses (Readable Registers, FC 0x03) 
STATUS_ADDR_A_VACUUM = 18
STATUS_ADDR_B_VACUUM = 19
STATUS_ADDR_SUPPLY_CURRENT = 20
STATUS_ADDR_TEMPERATURE = 23
STATUS_ADDR_PUMP_SPEED = 24

# Status Register Start Address and Total Count 
STATUS_ADDR_START = 18
NUM_STATUS_REGS = 7 # Addresses 18 through 24

class VG10Gripper:
    """
    High-level control and status mapping for the OnRobot VG10 gripper.
    """
    def __init__(self, modbus_client: ModbusTCPClient):
        self.modbus_client = modbus_client

    def _build_command_word(self, mode, vacuum_level):
        """
        Packs the Control Mode (high byte) and Target Vacuum (low byte) 
        into a single 16-bit integer.
        """
        # Input Validation: Target Vacuum should never exceed 80% 
        if vacuum_level > 80 and mode == MODE_GRIP:
            print(f"WARNING: Target vacuum clamped to 80% (VG10 limit).")
            vacuum_level = 80
        
        # V_final = (M << 8) | T
        shifted_mode = mode << 8 
        final_value = shifted_mode | vacuum_level
        
        return final_value

    def activate_suction(self, channel, vacuum_level=80):
        """Sends the Grip command (MODE_GRIP) to the specified channel."""
        address = VG10_CHANNEL_A_CTRL_ADDR if channel == 'A' else VG10_CHANNEL_B_CTRL_ADDR
        channel_name = f"Channel {channel}"
        
        command_value = self._build_command_word(MODE_GRIP, vacuum_level)
        
        print(f"--> Commanding {channel_name} to GRIP at {vacuum_level}% (Value: {command_value}).")
        
        success = self.modbus_client._safe_write_holding_register(address, command_value)
        
        if success:
            print(f"Status: {channel_name} Grip command sent successfully.")
        return success

    def deactivate_suction(self, channel):
        """Sends the Release command (MODE_RELEASE) to the specified channel."""
        address = VG10_CHANNEL_A_CTRL_ADDR if channel == 'A' else VG10_CHANNEL_B_CTRL_ADDR
        channel_name = f"Channel {channel}"

        # Release command word is 0x0000 (0)
        command_value = self._build_command_word(MODE_RELEASE, 0)
        
        print(f"--> Commanding {channel_name} to RELEASE (Value: {command_value}).")
        
        success = self.modbus_client._safe_write_holding_register(address, command_value)

        if success:
            print(f"Status: {channel_name} Release command sent successfully.")
        return success

    def read_status(self):
        """
        Reads a block of status registers and returns them as a translated dictionary, 
        applying necessary scaling.
        """
        # Read 7 registers starting at Address 18 
        raw_data = self.modbus_client._safe_read_holding_registers(STATUS_ADDR_START, NUM_STATUS_REGS)
        
        if raw_data is None:
            return {"Overall Status": "Communication Failure."}

        # ********** CORRECTION: Explicitly access index 0 **********
        # Correct Mapping of raw_data indices (0 through 6) to Registers 18 through 24
        raw_map = {
            'A_Vacuum_permille': raw_data[0], # Address 18 (Index 0). FIX: Extracting integer.
            'B_Vacuum_permille': raw_data[1], # Address 19 (Index 1)
            'Supply_Current_mA': raw_data[2], # Address 20 (Index 2)
            'Supply_Voltage_mV': raw_data[3], # Address 21 (Index 3)
            'Internal_5V_Voltage_mV': raw_data[4], # Address 22 (Index 4)
            'Temperature_c_x100': raw_data[5], # Address 23 (Index 5)
            'Pump_Speed_RPM': raw_data[6], # Address 24 (Index 6)
        }

        # Apply Scaling for human-readable output based on documentation 
        a_vacuum_pct = raw_map['A_Vacuum_permille'] / 10.0 # permille / 10 = percent
        b_vacuum_pct = raw_map['B_Vacuum_permille'] / 10.0
        temp_c = raw_map['Temperature_c_x100'] / 100.0 # 1/100 deg C / 100 = deg C
        supply_v = raw_map['Supply_Voltage_mV'] / 1000.0 # mV / 1000 = Volts
        pump_speed_rpm = raw_map['Pump_Speed_RPM']

        report = {
            "A_Vacuum": f"{a_vacuum_pct:.1f}%",
            "B_Vacuum": f"{b_vacuum_pct:.1f}%",
            "Current_Draw": f"{raw_map['Supply_Current_mA']} mA",
            "Supply_Voltage": f"{supply_v:.2f} V",
            "Temperature": f"{temp_c:.2f} °C",
            "Pump_Speed": f"{pump_speed_rpm} RPM",
        }
        
        # Operational Deduction
        pump_running = "Running" if pump_speed_rpm > 100 else "Stopped" # Assuming a minimal RPM threshold
        report['Pump_Operation'] = pump_running
        
        if a_vacuum_pct > 50:
            report['A_Operation'] = "Grip Success (High Vacuum)"
        elif a_vacuum_pct > 0:
            report['A_Operation'] = "Grip Attempt (Low Vacuum/Leak)"
        else:
            report['A_Operation'] = "Released/Idle"
        
        return report
    
    def read_vacuum_percent(self, channel=None):
        address = VG10_CHANNEL_A_CTRL_ADDR if channel == 'A' else VG10_CHANNEL_B_CTRL_ADDR
        regs = self.modbus_client._safe_read_holding_registers(address, 1)
        return (regs[0] & 0x00FF) if regs else None
    
    def grip_both(self, pct):
        if pct < 0: pct = 0
        if pct > 80: pct = 80
        value = (1 << 8) | int(pct)  # mode=1 (Grip) in high byte, target% in low byte
        # Write regs 0 and 1 in one shot
        resp = self.modbus_client.client.write_registers(0, [value, value], slave=self.modbus_client.slave_id)
        return not resp.isError()

def test():
    # NOTE: Set the IP address of your OnRobot Control Box (Gateway)
    GRIPPER_IP = '192.168.1.1' 
    MODBUS_PORT = 502

    print(f"--- Initializing OnRobot VG10 TCP Driver (IP: {GRIPPER_IP}:{MODBUS_PORT}) ---")
    
    # 1. Initialize the low-level communication client
    # The Slave ID (Unit ID) is typically 1 for the first device behind a gateway.
    modbus_comm = ModbusTCPClient(host=GRIPPER_IP, port=MODBUS_PORT, slave_id=65)
    
    if modbus_comm.connect():
        gripper = VG10Gripper(modbus_comm)

        try:
            # 1. Read initial status
            print("\n--- Reading Initial Status ---")
            status = gripper.read_status()
            for key, value in status.items():
                print(f"| {key.ljust(20)}: {value}")

            # 2. Activate suction on Channel A at 80% vacuum
            print("\n--- Activating Suction (Channel A at 80%) ---")
            # gripper.activate_suction('A', vacuum_level=80)
            gripper.grip_both(80)
            
            # Simulated delay for vacuum to build
            time.sleep(2)

            print("Probe: FC3 read control A (addr 0)")
            print(modbus_comm._safe_read_holding_registers(0, 1))  # expect [336] right after the write
            print("Probe: FC3 read control A (addr 0)")
            print(modbus_comm._safe_read_holding_registers(1, 1))

            print(gripper.read_vacuum_percent())
            print(modbus_comm._safe_read_holding_registers(24, 1))

            # 3. Read status to confirm vacuum build-up
            print("\n--- Reading Status After Activation ---")
            status = gripper.read_status()
            for key, value in status.items():
                print(f"| {key.ljust(20)}: {value}")

            print(modbus_comm._safe_read_holding_registers(24, 1))

            
            # 4. Deactivate suction on Channel A (Release)
            print("\n--- Deactivating Suction (Channel A) ---")
            gripper.deactivate_suction('A')
            gripper.deactivate_suction('B')
            
            time.sleep(1)

            # 5. Read final status
            print("\n--- Reading Final Status ---")
            status = gripper.read_status()
            for key, value in status.items():
                print(f"| {key.ljust(20)}: {value}")

        finally:
            print("\n--- Closing Modbus Connection ---")
            modbus_comm.close()
    else:
        print("Application terminated due to communication failure.")


# --- Execution Example ---
if __name__ == '__main__':
    test()