#! /bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import re


class OutputNode(Node):
    def __init__(self):
        super().__init__('output_node')
        self.subscription_ = self.create_subscription(
            msg_type=String,
            topic="/chatbot",
            callback=self.listener_callback,
            qos_profile=10
        )

        self.get_logger().info("Ouvindo ao /chatbot")

        self.publisher_ = self.create_publisher(
            msg_type = Float32MultiArray,
            topic = '/waypoints',
            qos_profile=10)

        self.pattern = r"\(\s*\d+(\.\d+)?\s*,\s*\d+(\.\d+)?\s*\)"

    def listener_callback(self, msg):
        matched = re.search(self.pattern, msg.data)

        if matched:
            self.x = float(matched.group(1))
            self.y = float(matched.group(2))

            points = Float32MultiArray()
            points.data.append(self.x)
            points.data.append(self.y)

            print(points)
            
            self.publisher_.publish(points)
        else:
            self.get_logger().info("Não encontrei os pontos")

        print(f"[RESPONSE] [CHATBOT] {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    output_node = OutputNode()

    rclpy.spin(output_node)

    output_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
