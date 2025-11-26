import rclpy
from rclpy.node import Node
import asyncio
import threading
from std_msgs.msg import String
import json
import random

class dummy_status_node(Node):
    
    def __init__(self):
        super().__init__('dummy_status_node')
        
        self.status = ['IDLE', 'MOVING', 'WAITING', 'CHARGING']
        self.current_status = self.status[0]
        self.battery_decreasing = [0.2, 0.01, 0.1]
        self.battery_increasing = [1, 2, 3]
        self.error = "None"
        self.loop = asyncio.new_event_loop()
        
        self.loop_thread = threading.Thread(target=self.start_async, daemon=True)
        self.loop_thread.start()
        
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.battery = 100.0
        
        self.create_timer(3.0, self.run_timer)
        
        return
    
    def run_timer(self):
        asyncio.run_coroutine_threadsafe(self.pub_dummy(), self.loop)
        return
    
    async def pub_dummy(self):
        
        self.battery -= self.battery_decreasing[int(random.random()) % 3]
        
        if self.current_status == self.status[3] and self.battery < 90.0:
            self.battery += self.battery_increasing[int(random.random()) % 3]
            
        elif self.current_status == self.status[3] and self.battery >=90.0:
            self.current_status = self.status[0]
            
        elif self.battery < 20.0:
            self.current_status = self.status[3]
            
        else:
            pick = (int(random.random()) % 2) + 1
            print(pick)
            self.current_status = self.status[pick] # 1 or 2
            
            
        if (int(random.random()) % 100) == 99:
            self.error = "Error"
        
        data = {
            "battery": round(self.battery, 4),
            "status": self.current_status,
            "error": self.error
        }
        
        msg = String()
        msg.data = json.dumps(data)
        
        await self.status_pub.publish(msg)
        
        self.get_logger().info(f"Published: {msg.data}")
        
        return
    
    def start_async(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        return
    
    async def async_shutdown(self):
        if self.client:
            await self.client.close()
            self.get_logger().info("HTTP Session Closed.")
        self.loop.stop()

    def destroy_node(self):
        future = asyncio.run_coroutine_threadsafe(self.async_shutdown(), self.loop)
        future.result()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    dummy_stat = dummy_status_node()
    
    rclpy.spin(dummy_stat)
    
    dummy_stat.destroy_node()
    rclpy.shutdown()
    
    return

if __name__ == '___main___':
    main()