import time
from pymodbus.client import ModbusTcpClient as ModbusClient
from pymodbus.exceptions import ModbusException

class ModbusTCPClient:
    """
    Handles low-level Modbus TCP communication over Ethernet.
    """
    def __init__(self, host, port=502, slave_id=65, timeout=1.0, use_dummy=False):
        self.slave_id = slave_id
        self.host = host
        self.port = port
        self.use_dummy = use_dummy
        # Use ModbusTcpClient for Ethernet connection
        if use_dummy:
            self.client = None
        else:
            self.client = ModbusClient(
                host=host,
                port=port,
                timeout=timeout
            )

    def connect(self):
        """Establishes the Ethernet connection."""
        if self.use_dummy:
            return True
        try:
            if self.client.connect():
                print(f"Modbus TCP client connected to {self.host}:{self.port}.")
                return True
            else:
                print(f"Failed to connect to Modbus TCP server at {self.host}:{self.port}.")
                return False
        except Exception as e:
            print(f"Error during TCP client connection: {e}")
            return False

    def close(self):
        """Closes the Ethernet socket connection."""
        if self.use_dummy:
            return True
        if self.client:
            self.client.close()

    def _safe_write_holding_register(self, address, value):
        """
        Writes a single 16-bit value to a Holding Register (FC 0x06). 
        Slave ID is still required if connecting through a gateway.
        """
        if self.use_dummy:
            return True
        
        if not self.client.connected:
            if not self.connect():
                return False
        
        try:
            # write_register uses FC 0x06 (Write Single Holding Register)
            response = self.client.write_register(address, value, slave=self.slave_id)
            
            if response.isError():
                print(f"Modbus Exception: Write failed at address {address}. Response: {response}")
                return False
            
            return True
            
        except ModbusException as e:
            print(f"Communication Error during write to address {address}: {e}")
            return False
        
    def _safe_write_holding_registers(self, address, values):
        """
        Writes multiple 16-bit values to Holding Registers (FC 0x10). 
        """
        if self.use_dummy:
            return True
        
        if not self.client.connected:
            if not self.connect():
                return False
        
        try:
            # write_registers uses FC 0x10 (Write Multiple Holding Registers)
            response = self.client.write_registers(address, values, slave=self.slave_id)
            
            if response.isError():
                print(f"Modbus Exception: Write failed at address {address}. Response: {response}")
                return False
            
            return True
            
        except ModbusException as e:
            print(f"Communication Error during write to address {address}: {e}")
            return False

    def _safe_read_holding_registers(self, address, count=1):
        """
        Reads one or more Holding Registers (FC 0x03) for status.
        """
        if self.use_dummy:
            return [0] * count
        
        if not self.client.connected:
            if not self.connect():
                return None
        
        try:
            # read_holding_registers uses FC 0x03 (Read Holding Registers)
            response = self.client.read_holding_registers(address, count=count, slave=self.slave_id)
            
            if response.isError():
                print(f"Modbus Exception: Read failed at address {address}. Response: {response}")
                return None
                
            if hasattr(response, 'registers') and response.registers:
                return response.registers
            else:
                print(f"Error: Received empty register list from address {address}.")
                return None
                
        except ModbusException as e:
            print(f"Communication Error during read from address {address}: {e}")
            return None