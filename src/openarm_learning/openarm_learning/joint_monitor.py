import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        
        self.subscription = self.create_subscription(
            JointState, #message type
            '/joint_states', # topic
            self.joint_state_callback, #callback
            10 # Qos depth
        )
    
    def joint_state_callback(self, msg):
        # print(msg.position)
        target_name = 'openarm_left_joint1'
        
        if target_name not in msg.name:
            self.get_logger().warning(
                f'{target_name} not found in JointState'
            )
            return
        
        index = msg.name.index(target_name)

        if index >= len(msg.position):
            self.get_logger(
                f'No position data for {target_name}'
            )
        
        pos = msg.name.index(target_name)
        self.get_logger().info(f'{target_name}:{msg.position[pos]} rad')
        

def main(args = None):
    rclpy.init(args = args)
    
    node = JointMonitor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()