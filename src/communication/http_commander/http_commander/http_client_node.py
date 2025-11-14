import rclpy
from rclpy.node import Node
import requests

class HttpClientNode(Node):
    def __init__(self):
        super().__init__('http_client_node')
        
        self.declare_parameter("server_url", "http://127.0.0.1")
        self.declare_parameter("api_key", "abcd")
        self.declare_parameter("server_port", 80)
        self.declare_parameter("timeout", 1.0)
        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.timeout = self.get_parameter("timeout").value
        
        self.timer = self.create_timer(5.0, self.send_request)
        self.get_logger().info("HTTP Client Node Started.")
        
    def send_request(self): #sort of heartbeat
        try:
            response = requests.get(self.url+":"+self.port, timeout=self.timeout)
            self.get_logger().info(f"Status: {response.status_code}")
            self.get_logger().info(f"Body: {response.text}")
        except requests.exceptions.Timeout as e:
            pass # heartbeat failed
        except Exception as e:
            self.get_logger().error(f"Request failed: {e}") # heartbeat failed
            
            
def main(args=None):
    rclpy.init(args=args)
    node = HttpClientNode()
    rclpy.spin(node)
    rclpy.spin_once()
    node.destroy_node()
    rclpy.shutdown()