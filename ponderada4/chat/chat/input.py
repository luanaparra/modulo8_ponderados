#! /bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(
            msg_type=String, 
            topic='/llm', 
            qos_profile=10
        )
        
        self.get_logger().info('Estou pronto para receber perguntas...')
    
    def run(self):
        try:
            while rclpy.ok():
                user_input = input("Faça uma pergunta: ")
                if user_input.lower() == "exit":
                    break
                self.publish_command(user_input)
        except KeyboardInterrupt:
            pass
    
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publicando '{command}'")

def main(args=None):
    rclpy.init(args=args)
    input_node = InputNode()
    
    input_node.run()
    
    input_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()