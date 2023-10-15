# Author : Marko Mizdrak
# Date    : 15/10/2023
# License : MIT

import rclpy
from rclpy.node import Node
from pydrivingsim import Vehicle

# TODO get message definitions
from std_msgs.msg import String


class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')
        self.subscription = self.create_subscription(
            String, # TODO change to something better
            'control_command',
            self.control_command_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(String, 'vehicle_state', 10)
        
        self.vehicle_state = String() # message

        self.vehicle = Vehicle()

        # TODO set car start pose
        
    def control_command_callback(self, msg):
        self.get_logger().info(f'Received control command: Steering Angle = {msg.steering_angle}, Accelerator Pedal Position = {msg.accelerator_pedal}')
        print(msg)

        # TODO take in messages and convert to action suitable for vehicle model

        # interface with vehicle as follows
        action = (0.1, 0) 
        self.vehicle.control(action)
        self.vehicle.update()

        vehicle_state, _ = self.vehicle.get_state()



        # TODO convert state to correct form to publish message 

        state_string = vehicle_state.tostring()
        
        self.publisher_.publish(state_string)
        self.get_logger().info(f'Published vehicle state: Speed = {self.vehicle_state.speed}, Position = {self.vehicle_state.position}')

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = VehicleNode()
    rclpy.spin(vehicle_node)
    vehicle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



