import time

from typeguard import value
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
NUM_STATUS_REGS = 7 

class VGC10GripperBase:
    """
    High-level control and status mapping for the OnRobot VG10 gripper.
    """
    def __init__(self, modbus_client: ModbusTCPClient):
        self.modbus_client = modbus_client
        self.vacuum_percent = 0

    def _build_command_word(self, mode, vacuum_level):
        """
        Packs the Control Mode (high byte) and Target Vacuum (low byte) 
        into a single 16-bit integer.
        """
        if vacuum_level > 80 and mode == MODE_GRIP:
            print(f"WARNING: Target vacuum clamped to 80% (VG10 limit).")
            vacuum_level = 80
        
        shifted_mode = mode << 8 
        final_value = shifted_mode | vacuum_level
        
        return final_value
    
    def activate_suction(self, channel, vacuum_level=80):
        """Sends the Grip command (MODE_GRIP) to the specified channel."""
        vacuum_level = max(0, min(80, int(vacuum_level)))
        command_value = self._build_command_word(MODE_GRIP, vacuum_level)
        if not channel in ['A', 'B']:
            success = self.modbus_client._safe_write_holding_register(0, [command_value, command_value])
            if success:
                print(f"--> Commanding BOTH Channels to GRIP at {vacuum_level}% (Value: {command_value}).")
                print(f"Status: Both Channels Grip command sent successfully.")
                self.vacuum_percent = vacuum_level
                return True
            return False             

        address = VG10_CHANNEL_A_CTRL_ADDR if channel == 'A' else VG10_CHANNEL_B_CTRL_ADDR
        channel_name = f"Channel {channel}"        
        print(f"--> Commanding {channel_name} to GRIP at {vacuum_level}% (Value: {command_value}).")
        
        success = self.modbus_client._safe_write_holding_register(address, command_value)
        
        if success:
            print(f"Status: {channel_name} Grip command sent successfully.")
        return success

    def deactivate_suction(self):
        """Sends the Release command (MODE_RELEASE) to the specified channel."""
        command_value = self._build_command_word(MODE_RELEASE, 0)
        success = self.modbus_client._safe_write_holding_register(0, [command_value, command_value])
        
        if success:
            print(f"Status: Both Channels Release command sent successfully.")
            self.vacuum_percent = 0
            return True
        else:
            return False
        
    def set_to_idle(self, channel):
        """Sends the Idle command (MODE_IDLE) to the specified channel."""
        command_value = self._build_command_word(MODE_IDLE, 0)
        if not channel in ['A', 'B']:
            success = self.modbus_client._safe_write_holding_register(0, [command_value, command_value])
            if success:
                print(f"--> Commanding BOTH Channels to IDLE (Value: {command_value}).")
                print(f"Status: Both Channels Idle command sent successfully.")
                return True
            return False

        address = VG10_CHANNEL_A_CTRL_ADDR if channel == 'A' else VG10_CHANNEL_B_CTRL_ADDR
        channel_name = f"Channel {channel}"        
        print(f"--> Commanding {channel_name} to IDLE (Value: {command_value}).")
        success = self.modbus_client._safe_write_holding_register(address, command_value)
        
        if success:
            print(f"Status: {channel_name} Idle command sent successfully.")
        return success

    def read_status_individually(self):
        """
        Reads all status registers (Addresses 18 through 24) one by one 
        and translates the data using scaling.
        """
        
        # --- 1. Read Raw Data One-by-One (FC 0x03) ---
        ## TODO: This is inefficient; consider batch reading in future.
        
        def _read_single(address):
            regs = self.modbus_client._safe_read_holding_registers(address, 1)
            return regs[0] if regs else None

        first = _read_single(STATUS_ADDR_A_VACUUM)
        if first is None:
            return {"Overall_Status": "Communication Failure"}

        raw = {
            "A_Vacuum_permille": _read_single(STATUS_ADDR_A_VACUUM),
            "B_Vacuum_permille": _read_single(STATUS_ADDR_B_VACUUM),
            "Supply_Current_mA": _read_single(STATUS_ADDR_SUPPLY_CURRENT),
            "Reserved_21": _read_single(21),
            "Internal_5V_Voltage_mV": _read_single(22),
            "Temperature_x100C": _read_single(STATUS_ADDR_TEMPERATURE),
            "Pump_Speed_RPM": _read_single(STATUS_ADDR_PUMP_SPEED),
        }

        # --- 2. Decoding and Scaling the Information ---
        
        # The information is NOT in raw bytes at this stage; pymodbus has already 
        # decoded the raw Modbus response frame (which does use bytes) into a list 
        # of standard 16-bit Python integers.
        
        # Decoding process is simply applying the manufacturer's scaling factor:
        
        # Vacuum: Permille (1/1000) -> Percent (Divide by 10) 
        a_vac_pct = raw["A_Vacuum_permille"] / 10.0
        b_vac_pct = raw["B_Vacuum_permille"] / 10.0
        temp_c = raw["Temperature_x100C"] / 100.0
        supply_v = raw["Internal_5V_Voltage_mV"] / 1000.0
        pump_rpm = raw["Pump_Speed_RPM"]
        current_mA = raw["Supply_Current_mA"]

        report = {
            "A_Vacuum": f"{a_vac_pct:.1f}%",
            "B_Vacuum": f"{b_vac_pct:.1f}%",
            "Current_Draw": f"{current_mA} mA",
            "Supply_Voltage": f"{supply_v:.2f} V",
            "Temperature": f"{temp_c:.2f} °C",
            "Pump_Speed": f"{pump_rpm} RPM",
            "Pump_Operation": "Running" if pump_rpm > 100 else "Stopped",
            "A_Operation": (
                "Grip Success (High Vacuum)" if a_vac_pct > 50
                else "Grip Attempt (Low Vacuum/Leak)" if a_vac_pct > 0
                else "Released/Idle"
            ),
            "Raw": raw
        }
        return report
    


#####################################################################
############################## TESTING ##############################
#####################################################################
def test():
    # NOTE: Set the IP address of your OnRobot Control Box (Gateway)
    GRIPPER_IP = '192.168.1.1' 
    MODBUS_PORT = 502

    print(f"--- Initializing OnRobot VG10 TCP Driver (IP: {GRIPPER_IP}:{MODBUS_PORT}) ---")
    
    # 1. Initialize the low-level communication client
    # The Slave ID (Unit ID) is typically 1 for the first device behind a gateway.
    modbus_comm = ModbusTCPClient(host=GRIPPER_IP, port=MODBUS_PORT, slave_id=65)
    
    if modbus_comm.connect():
        gripper = VGC10GripperBase(modbus_comm)

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

            
            # 4. Deactivate suction 
            gripper.deactivate_suction()
            
            time.sleep(1)

            # 5. Read final status
            print("\n--- Reading Final Status ---")
            status = gripper.read_status()
            for key, value in status.items():
                print(f"| {key.ljust(20)}: {value}")

        finally:
            # --- SAFETY AND CLEANUP BLOCK ---
            print("\n--- Initiating Safety Shutdown (Sending Release) ---")
            
            # 1. Send Release command to critical channel(s)
            # We target Channel A to ensure the vacuum is actively dropped.
            # We suppress the output and rely on the client's internal error handling.
            gripper.deactivate_suction() 
            
            # Optional: Add a small delay to ensure the command is sent before closing the socket
            time.sleep(0.1) 
            
            # 2. Close the connection
            print("--- Closing Modbus Connection ---")
            modbus_comm.close()
    else:
        print("Application terminated due to communication failure.")


# --- Execution Example ---
if __name__ == '__main__':
    test()