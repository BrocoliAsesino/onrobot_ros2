import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter

from control_msgs.action import GripperCommand
from onrobot_interfaces.srv import VGC10Status
from vgc10driver.modbus_tcp_client import ModbusTCPClient
from vgc10driver.vgc10_gripper_base import VGC10GripperBase


class VGC10ROS2Driver(Node):
    def __init__(self, node_name: str, gripper_base: VGC10GripperBase):
        super().__init__(node_name)
        self.gripper_base = gripper_base
        
        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.create_services()
        
        self.get_logger().info('VGC10 ROS2 Driver initialized')
        
    def create_services(self):
        """Create all ROS services and action servers"""
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/onrobot_vgc10_controller/gripper_command',
            self.execute_action_callback,
            callback_group=self.action_callback_group
        )
        
        self._status_service = self.create_service(
            VGC10Status,
            'onrobot_vgc10/get_status',
            self.get_status_callback,
            callback_group=self.service_callback_group
        )

    def get_status_callback(self, request, response):
        try:
            status = self.gripper_base.read_status_individually()
            
            if 'Overall Status' in status and status['Overall Status'] == "Communication Failure.":
                response.success = False
                response.message = "Communication failure with gripper"
                return response
            
            response.vacuum_a = float(status['A_Vacuum'].strip('%'))
            response.vacuum_b = float(status['B_Vacuum'].strip('%'))
            
            response.current_draw = float(status['Current_Draw'].split()[0])
            response.temperature = float(status['Temperature'].split()[0])
            response.pump_speed = int(status['Pump_Speed'].split()[0])
            
            if 'Supply_Voltage' in status:
                response.supply_voltage = float(status['Supply_Voltage'].split()[0])
            
            response.pump_state = status['Pump_Operation']
            response.channel_a_state = status['A_Operation']
            
            response.success = True
            response.message = "Status read successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Error reading status: {str(e)}"
            self.get_logger().error(f'Status service error: {str(e)}')
        
        return response
    
    async def execute_action_callback(self, goal_handle):
        try:
            command = goal_handle.request
            position = command.command.position  
            
            vacuum_level = int(position)
            
            if vacuum_level > 0:
                success = self.gripper_base.activate_suction('AB', vacuum_level)
            else:
                success = self.gripper_base.deactivate_suction()
            
            status = self.gripper_base.read_status_individually()
            
            result = GripperCommand.Result()
            
            current_vacuum = float(status['A_Vacuum'].strip('%')) 
            result.position = current_vacuum
            result.effort = float(status['Current_Draw'].split()[0])  # Current draw as effort
            result.stalled = status['Pump_Operation'] == "Stopped"
            result.reached_goal = success

            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f'Action execution error: {str(e)}')
            goal_handle.abort()
            return GripperCommand.Result()

def main():
    rclpy.init()
    
    try:
        # Get parameters from launch file or command line
        node = rclpy.create_node('vgc10_parameter_node')
        ip_address = node.declare_parameter('ip_address', '192.168.1.1').value
        port = node.declare_parameter('port', 502).value
        slave_id = node.declare_parameter('slave_id', 65).value
        use_dummy = node.declare_parameter('use_dummy', False)
        
        # Initialize gripper
        modbus_client = ModbusTCPClient(host=ip_address, port=port, slave_id=slave_id,
                                        use_dummy=use_dummy)

        if not modbus_client.connect():
            raise RuntimeError("Failed to connect to gripper")
            
        gripper_base = VGC10GripperBase(modbus_client)
        vgc10_driver = VGC10ROS2Driver("VGC10_Gripper_Driver_Node", gripper_base)
        
        executor = MultiThreadedExecutor()
        executor.add_node(vgc10_driver)
        
        try:
            executor.spin()
        finally:
            gripper_base.deactivate_suction()
            modbus_client.close()
            vgc10_driver.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"Error initializing driver: {str(e)}")
        rclpy.shutdown()

if __name__ == "__main__":
    main()




