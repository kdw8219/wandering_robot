import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import asyncio
import json
from std_msgs.msg import String
import threading
from queue import Queue
import socket

import grpc
import grpc_commander.grpc.generated.robot_gateway_api_pb2 as pb
import grpc_commander.grpc.generated.robot_gateway_api_pb2_grpc as pb2_grpc


#TODO : async 배제, threading 위주로 코드 개선

class GrpcClientNode(Node):

    def __init__(self):
        super().__init__('grpc_client_node')

        # PARAMETERS
        self.declare_parameter("server_url", "127.0.0.1")
        self.declare_parameter("server_port", 50051)
        self.declare_parameter("robot_id", "test_robot_1")
        self.declare_parameter("robot_secret", "abcdzxcv")
        self.declare_parameter("request_expired", 3.0)
        self.declare_parameter("access_secret_key", "abcdefg")
        
        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.robot_id = self.get_parameter("robot_id").value
        self.robot_secret = self.get_parameter("robot_secret").value
        self.request_expired = self.get_parameter("request_expired").value
        self.access_secret_key = self.get_parameter("access_secret_key").value
        
        # gRPC 채널 생성 (동기식, keepalive 포함 가능)
        self.channel = grpc.insecure_channel(
            self.url + ':' + self.port,
            options=[
                ('grpc.keepalive_time_ms', 10000),
                ('grpc.keepalive_timeout_ms', 5000),
                ('grpc.http2.max_pings_without_data', 0),
                ('grpc.enable_retries', 1)
            ]
        )

        # Stub 생성
        self.stub = pb2_grpc.RobotApiGatewayStub(self.channel)

        # TOKENS
        self.on_refreshing = False
        self.access_token = ""
        self.refresh_token = ""

        # Async components
        self.loop = asyncio.new_event_loop()
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
        
        self.get_logger().info("GRPC Client Node Started.")

    def status_callback(self, msg: String):
        task = {
            "func": "status",
            "payload" : json.loads(msg.data)
        }
        
        self.queue.put(task)
        
    def pos_callback(self, msg: String):
        task = {
            "func" : "pos",
            "payload" : json.loads(msg.data)
        }
        
        self.queue.put(task)

    def workerThread(self):
        while True: 
            task = self.queue.get()
            try:
                self.get_logger().info(f"Trying function: {task['func']}")
                if 'heartbeat' in task['func']:
                    request = pb.HeartbeatRequest(
                        robot_id=task['payload']['robot_id'],
                    )
                    response = self.stub.Heartbeat(request, timeout=3.0)
                    
                    self.get_logger().info(f"Heartbeat success? : {response.success}. Result: {response.result}.")
                elif 'status' in task['func']:
                    request = pb.StatusRequest(
                        robot_id=self.robot_id,
                        battery = float(task['payload']['battery']),
                        status = task['payload']['status'],
                        error = task['payload']['error'],
                    )
                    response = self.stub.Status(request, timeout=3.0)
                    self.get_logger().info(f"Status success? : {response.success}. Result: {response.result}.")
                elif 'pos' in task['func']:
                    request = pb.PosRequest(
                        robot_id=self.robot_id,
                        x = float(task['payload']['x']),
                        y = float(task['payload']['y']),
                        z = float(task['payload']['z']),
                        orig_x = float(task['payload']['orientation']['x']),
                        orig_y = float(task['payload']['orientation']['y']),
                        orig_z = float(task['payload']['orientation']['z']),
                        orig_w = float(task['payload']['orientation']['w']),
                        linear_speed = float(task['payload']['linear_speed']),
                        angular_speed = float(task['payload']['angular_speed']),
                    )
                    response = self.stub.Pos(request, timeout=3.0)
                    self.get_logger().info(f"Pos success? : {response.success}. Result: {response.result}.")
                else :
                    self.get_logger().warning(f"Unknown task function: {task['func']}")
                    continue
            
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
            
            #self.queue.task_done() # 필요 없음. join을 쓰지 않기 때문.

    def heartbeat(self):
        task = {
            "func": "heartbeat",
            "payload" : {
                'robot_id': self.robot_id,
            }
        }
        
        self.queue.put(task)
        
        return

    # =======================================================
    # Timer callback
    # =======================================================
    def timer_callback(self):
        if self.login_tried is False:
            self.try_login_once()
            return
        
        #run additional (like jwt)
        
        #Not Connected
        if self.login_tried is False:
            return
        
        self.heartbeat()
        
    # =======================================================
    # Login Once
    # =======================================================
    def try_login_once(self):
        if self.login_tried is True:
            return

        try:
            request = pb.LoginRequest(
                robot_id=self.robot_id,
                robot_secret=self.robot_secret
            )
            
            self.get_logger().info(f"Robot ID? : {self.robot_id}.")
            self.get_logger().info(f"Robot Secret? : {self.robot_secret}.")
            
            response = self.stub.Login(request, timeout=3.0)

            self.get_logger().info(f"Login success? : {response.success}.")
            
            if response.success is False:
                self.get_logger().error("Login failed. Check robot_id and robot_secret.")
                return
            
            self.access_token = response.access_token
            self.refresh_token = response.refresh_token
            
            # loop thread should run after login
            self.loop_thread.start()
            
            self.login_tried = True
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.get_logger().error(f"Unexpected error: {e}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON. Error: {e}")


    def get_robot_ip(self):
        server_addr=(self.server_ip, 7777)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(server_addr)
            return s.getsockname()[0]
        finally:
            s.close()

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
    node = GrpcClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
