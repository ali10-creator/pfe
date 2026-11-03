#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import sys

class RadiusChanger(Node):
    def __init__(self):
        super().__init__('radius_changer')
        
    def change_radius(self, radius):
        # Create client for parameter service
        param_client = self.create_client(
            SetParameters, 
            '/circular_smc_node/set_parameters'
        )
        
        # Wait for service
        if not param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Parameter service not available')
            return
            
        # Create parameter
        param = Parameter()
        param.name = 'radius'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = radius
        
        # Send request
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Successfully changed radius to {radius}')
        else:
            self.get_logger().error('Failed to change radius')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 test_radius_change.py <radius>")
        print("Example: python3 test_radius_change.py 3.0")
        return
        
    try:
        radius = float(sys.argv[1])
    except ValueError:
        print("Error: Radius must be a number")
        return
        
    rclpy.init(args=args)
    node = RadiusChanger()
    node.change_radius(radius)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 