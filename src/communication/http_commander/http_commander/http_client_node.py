import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import requests
import asyncio
import json
from std_msgs.msg import String
import threading
from queue import Queue

#TODO : async 배제, threading 위주로 코드 개선

class HttpClientNode(Node):

    def __init__(self):
        super().__init__('http_client_node')

        # PARAMETERS
        self.declare_parameter("server_url", "http://127.0.0.1")
        self.declare_parameter("server_port", 80)
        self.declare_parameter("robot_id", "abcd")
        self.declare_parameter("robot_secret", "abcdzxcv")
        self.declare_parameter("request_expired", 3.0)
        self.declare_parameter("access_secret_key", "abcdefg")

        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.robot_id = self.get_parameter("robot_id").value
        self.robot_secret = self.get_parameter("robot_secret").value
        self.request_expired = self.get_parameter("request_expired").value
        self.access_secret_key = self.get_parameter("access_secret_key").value

        # TOKENS
        self.on_refreshing = False
        self.access_token = ""
        self.refresh_token = ""

        # Async components
        self.loop = asyncio.new_event_loop()
        self.client: requests.Session | None = None
        self.login_tried = False

        # ROS timer
        self.timer = self.create_timer(3, self.timer_callback)
        
        # thread task queue, Thread safe
        self.queue = Queue()
        
        # thread
        self.loop_thread = threading.Thread(target=self.workerThread, daemon=True)
        
        # subscribe gazebo odometry
        self.create_subscription(
            String,
            '/robot/status',
            self.status_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        # subscribe gazebo odometry
        self.create_subscription(
            String,
            '/robot/pos',
            self.pos_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("HTTP Client Node Started.")

    def status_callback(self, msg: String):
        task = {
            "url": f"{self.url}:{self.port}/api/robots/status/",
            "payload" : json.loads(msg.data)
        }
        
        self.queue.put(task)
        
        self.heartbeat() # status is periodical data so, can be handled in a single thread
        
    def pos_callback(self, msg: String):
        task = {
            "url": f"{self.url}:{self.port}/api/robots/pos/",
            "payload" : json.loads(msg.data)
        }
        
        self.queue.put(task)
        
        pass

    def workerThread(self):
        while True: 
            task = self.queue.get()
            
            task_string = "temp"
            if 'heartbeat' in task['url']:
                task_string = 'heartbeat'
            elif 'status' in task['url']:
                task_string = 'status'
            elif 'pos' in task['url']:
                task_string = 'position'
            
            self.get_logger().info(f"Trying URL: {task['url']}")
            
            try:
                resp = requests.post(task['url'], json=task['payload'], timeout=(1, 1)) 
                resp.raise_for_status()
                self.get_logger().info(f"{task_string} processing Success. + {resp.text}")

            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
            
            self.queue.task_done()
    
    def heartbeat(self):
        task = {
            "url": f"{self.url}:{self.port}/api/robots/heartbeat/",
            "payload" : {
                'robot_id': self.robot_id
            }
        }
        
        self.queue.put(task)
        
        return

    # =======================================================
    # Timer callback
    # =======================================================
    def timer_callback(self):
        if self.client is None:
            self.initialize_session()
            return

        if not self.login_tried:
            self.try_login_once()
            return
        
        #run additional (like jwt)
        
        #Not Connected
        if self.client is None or self.login_tried is None:
            return
        
        self.heartbeat()
        
    # =======================================================
    # Create aiohttp session
    # =======================================================
    def initialize_session(self):
        if self.client is None:
            try:
                self.client = requests.Session()
                self.get_logger().info("ClientSession initialized.")
            except Exception as e:
                self.get_logger().error(f"ClientSession init failed: {e}")


    # =======================================================
    # Login Once
    # =======================================================
    def try_login_once(self):
        if self.login_tried or self.client is None:
            return

        url = f"{self.url}:{self.port}/api/robots/login/"
        self.get_logger().info(f"Trying login: {url}")
        payload = {
            'robot_id': self.robot_id,
            'robot_secret': self.robot_secret
        }

        try:
            resp = self.client.post(url, json=payload, timeout=(1, 1))
            resp.raise_for_status()

            self.get_logger().info(f"Login success + {resp.text}.")
            self.login_tried = True
            
            json_data = json.loads(resp.text)
            self.access_token = json_data['access_token']
            self.refresh_token = json_data['refresh_token']
            
            # loop thread should run after login
            self.loop_thread.start()
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.get_logger().error(f"Unexpected error: {e}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON. Error: {e}")


    # =======================================================
    # Shutdown (node destroy)
    # =======================================================
    def shutdown(self):
        if self.client:
            self.client.close()
            self.get_logger().info("HTTP Session Closed.")
        

# ================================================================
# MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = HttpClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
