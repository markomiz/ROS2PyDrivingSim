# Author : Marko Mizdrak
# Date    : 15/10/2023
# License : MIT

import rclpy
from rclpy.node import Node
from pydrivingsim import Vehicle

# TODO get message definitions
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import math
import tf.transformations as tf_trans
from geometry_msgs.msg import Pose, Twist, Quaternion

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
        
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        
        self.vehicle_state = String() # message

        self.vehicle = Vehicle()

        # TODO set car start initial state if desired...
        
    def control_command_callback(self, msg):
        
        # TODO take in messages and convert to action suitable for vehicle model

        # interface with vehicle as follows
        action = (0.1, 0) 
        self.vehicle.dt = 0.001 # how long to execute the action
        self.vehicle.control(action) # this sets the action
        self.vehicle.update() # this integrates forward using the vehicle model with the specified action for the dt

        vehicle_state, _ = self.vehicle.get_state()

        odom = Odometry()
        # Set position
        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]

        # Set orientation
        quaternion = tf_trans.quaternion_from_euler(0, 0, state[2])
        odom.pose.pose.orientation = Quaternion(*quaternion)

        # Set linear velocity
        odom.twist.twist.linear.x = state.v * math.cos(state[2])
        odom.twist.twist.linear.y = state.v * math.sin(state[2])

        self.publisher_.publish(odom)
        

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = VehicleNode()
    rclpy.spin(vehicle_node)
    vehicle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



