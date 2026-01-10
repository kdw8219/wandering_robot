import json
import queue
import threading

class RobotController():
    def __init__(self, queue: queue.Queue):
        self.queue = queue
        self.worker = threading.Thread(target = self.workingThread)
        self.stop_event = threading.Event()
    
    def __del__(self):
        self.stop_event.set()
        self.worker.join()
        
    def run(self):
        self.worker.start()
        
    def workingThread(self):
        print("Robot Controller thread start!")
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout = 0.1) #block 아님?

                if isinstance(item, bytes):
                    item = item.decode("utf-8")
                if isinstance(item, str):
                    try:
                        item = json.loads(item)
                    except json.JSONDecodeError:
                        print(f"Invalid queue item (not JSON): {item}")
                        self.queue.task_done()
                        continue

                command = item.get("command") if isinstance(item, dict) else None
                print(f"get Item!{command}")
                
                #send to robot controller logic here
                
                self.queue.task_done()
            except queue.Empty:
                pass
