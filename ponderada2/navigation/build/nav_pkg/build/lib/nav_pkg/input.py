#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

intent_dict = {
    r"\bponto\s+x\b": "x",
    r"\bponto\s+y\b": "y",
    r"\bponto\s+w\b": "w", # Agora usamos diretamente a captura como chave
}

point_dict = {
    "x": [7.0, 7.0, 7.0],
    'y': [0.0, 0.0, 0.0],
    'w': [1.0, 0.0, 0.0],
}

class ChatbotPub(Node):
    def __init__(self):
        super().__init__('positions')
        self.publisher_ = self.create_publisher(Pose, 'positions', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def timer_callback(self):
        msg = Pose()
        comando = input("Digite algo: ")
        for chave, intencao in intent_dict.items():
            pattern = re.compile(chave)
            captura = pattern.findall(comando)
            if captura:
                ponto = point_dict[intencao]
                msg.position.x, msg.position.y, msg.position.z = ponto
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing Pose: x={}, y={}, z={}'.format(msg.position.x, msg.position.y, msg.position.z))
                break  # Saia do loop assim que encontrar uma correspondênciadef main(args=None):

def main(args=None):
    rclpy.init()
    chat_bot = ChatbotPub()
    rclpy.spin(chat_bot)
    chat_bot.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()