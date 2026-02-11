import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request
import threading

app = Flask(__name__)

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.pub = self.create_publisher(String, '/n8n_command', 10)

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {cmd}")

def main():
    rclpy.init()
    node = CommandNode()

    def spin():
        rclpy.spin(node)

    threading.Thread(target=spin, daemon=True).start()

    @app.route('/command', methods=['POST'])
    def handle():
        data = request.json
        cmd = data.get("command", "none")
        node.send_command(cmd)
        return {"status": "ok"}

    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
