import rclpy
from rclpy.node import Node
import requests
import time

class HttpClientNode(Node):
    def __init__(self):
        super().__init__('http_client_node')
        
        self.declare_parameter("server_url", "http://127.0.0.1")
        self.declare_parameter("server_port", 80)
        self.declare_parameter("robot_id", "abcd")
        self.declare_parameter("request_expired", 3.0)
        
        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.request_expired = self.get_parameter("request_expired").value
        self.robot_id = self.get_parameter("robot_id".value)
        
        self.access_token = ""
        self.refresh_token = ""
        
        self.jwt_timer = self.create_timer(5.0, self.send_request)
        self.get_logger().info("HTTP Client Node Started.")
        
        self.login_timer = self.create_timer(5.0, self.try_login_once)
        self.login_tried = False
        
    def try_login_once(self):
        
        if self.login_tried == True:
            return
        
        try: # use login model
            response = requests.post(self.url+":"+self.port, timeout=self.request_expired)
            self.get_logger().info(f"Status: {response.status_code}")
            self.get_logger().info(f"Body: {response.text}")
            
            self.login_tried = True
        except requests.exceptions.Timeout as e:
            self.login_tried = False
            self.get_logger().error(f"Request Timeout: {e}") # heartbeat failed
        except Exception as e:
            self.login_tried = False
            self.get_logger().error(f"Request failed: {e}") # heartbeat failed
        
        
    def send_request(self): #sort of heartbeat
        try:
            response = requests.get(self.url+":"+self.port, timeout=self.request_expired)
            self.get_logger().info(f"Status: {response.status_code}")
            self.get_logger().info(f"Body: {response.text}")
        except requests.exceptions.Timeout as e:
            self.get_logger().error(f"Request Timeout: {e}") # heartbeat failed
        except Exception as e:
            self.get_logger().error(f"Request failed: {e}") # heartbeat failed
            
            
def main(args=None):
    rclpy.init(args=args)
    node = HttpClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()