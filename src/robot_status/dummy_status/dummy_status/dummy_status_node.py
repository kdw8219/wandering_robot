import rclpy
from rclpy.node import Node
import asyncio
import threading
from std_msgs.msg import String
import json
import random
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from concurrent.futures import ThreadPoolExecutor
from queue import Queue

#TODO : async 배제, threading 위주로 코드 개선
#rclpy가 thread safe하지 않고, 비동기 동작이 2개 이상 동시에 동작하게 될 경우에 

class dummy_status_node(Node):
    
    def __init__(self):
        super().__init__('dummy_status_node')
        
        self.status = ['IDLE', 'MOVING', 'WAITING', 'CHARGING']
        self.current_status = self.status[0]
        self.battery_decreasing = [0.2, 0.01, 0.1]
        self.battery_increasing = [1, 2, 3]
        self.error = "None"
        self.loop = asyncio.new_event_loop()
        
        self.status_pub = self.create_publisher(String, '/robot/status', qos_profile_sensor_data)
        self.pos_pub = self.create_publisher(String, '/robot/pos', qos_profile_sensor_data)
        
        self.battery = 100.0
        
        
        self.queue = Queue()
        
        self.loop_thread = threading.Thread(target=self.workerThread, daemon=True)
        self.loop_thread.start()
                
        # subscribe gazebo odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=10
        )
        
        self.robot_state = {
            "x": 0,
            "y": 0,
            "z": 0,
            "orientation": {
                "x": 0, "y": 0, "z": 0, "w": 0
            },
            "linear_speed": 0,
            "angular_speed": 0,
        }
        
        self.create_timer(3.0, self.run_timer)
        
        return
    
    def run_timer(self):
        self.pub_dummy()
        return
    
    def pub_dummy(self):
        
        self.battery -= self.battery_decreasing[int(random.random()) % 3]
        
        if self.current_status == self.status[3] and self.battery < 90.0:
            self.battery += self.battery_increasing[int(random.random()) % 3]
            
        elif self.current_status == self.status[3] and self.battery >=90.0:
            self.current_status = self.status[0]
            
        elif self.battery < 20.0:
            self.current_status = self.status[3]
            
        else:
            pick = (int(random.random()) % 2) + 1
            self.current_status = self.status[pick] # 1 or 2
            
            
        if (int(random.random()) % 100) == 99:
            self.error = "Error"
        
        task = { 
            'task_name': 'status',
            'payload' : {
                "battery": round(self.battery, 4),
                "status": self.current_status,
                "error": self.error
            }
        }
        
        self.get_logger().info('create dummy status')
        self.queue.put(task)
        
        return
    
    def check_position_changed(self, msg) -> bool:
        if (abs(self.robot_state["x"] - msg.pose.pose.position.x) < 0.001 
            and abs(self.robot_state["y"] - msg.pose.pose.position.y) < 0.001 
            and abs(self.robot_state["z"] - msg.pose.pose.position.z) < 0.001
            and abs(self.robot_state['orientation']['x'] - msg.pose.pose.orientation.x) < 0.001
            and abs(self.robot_state['orientation']['y'] - msg.pose.pose.orientation.y) < 0.001
            and abs(self.robot_state['orientation']['z'] - msg.pose.pose.orientation.z) < 0.001
            and abs(self.robot_state['orientation']['w'] - msg.pose.pose.orientation.w) < 0.001
            and abs(self.robot_state["linear_speed"] - msg.twist.twist.linear.x) < 0.001
            and abs(self.robot_state["angular_speed"] - msg.twist.twist.angular.z) < 0.001) :
            return True
            
        return False
    
    def process_odom(self, msg):
        
        if self.check_position_changed(msg) is True:
            return False
        
        # position
        self.robot_state["x"] = msg.pose.pose.position.x
        self.robot_state["y"] = msg.pose.pose.position.y
        self.robot_state["z"] = msg.pose.pose.position.z
        
        # orientation
        self.robot_state["orientation"] = {
            "x": msg.pose.pose.orientation.x,
            "y": msg.pose.pose.orientation.y,
            "z": msg.pose.pose.orientation.z,
            "w": msg.pose.pose.orientation.w,
        }

        # speed
        self.robot_state["linear_speed"] = msg.twist.twist.linear.x
        self.robot_state["angular_speed"] = msg.twist.twist.angular.z

        self.get_logger().info(f"Robot pose: {self.robot_state}")
        
        return True
        # pos = String()
        # pos.data = json.dumps(self.robot_state)
        
        # self.pos_pub.publish(pos)
    
    def odom_callback(self, msg: Odometry):
        
        if False == self.process_odom():
            self.get_logger.info('Any changes in odometry data')
            return
        
        task = {
            'task_name': 'pos',
            'payload' : self.robot_state
        }
        
        self.queue.put(task)
        
    
    def workerThread(self):
        
        while True: 
            task = self.queue.get()
            
            msg = String()
            msg.data = json.dumps(task['payload'])
            
            try:
                if task['task_name'] == 'status':
                    self.status_pub.publish(msg)
                elif task['task_name'] == 'pos':
                    self.pos_pub.publish(msg)
                else:
                    self.get_logger().error(f'unsupported task name{task['task_name']}')
                    continue
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.get_logger().error(f"Unexpected error: {e}")
            
            
            self.get_logger().info(f"Published: {task['task_name']}")
    
    

def main(args=None):
    rclpy.init(args=args)
    
    dummy_stat = dummy_status_node()
    
    rclpy.spin(dummy_stat)
    
    dummy_stat.destroy_node()
    rclpy.shutdown()
    
    return

if __name__ == '___main___':
    main()

