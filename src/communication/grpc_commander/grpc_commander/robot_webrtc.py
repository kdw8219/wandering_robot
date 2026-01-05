import queue
import threading

class RobotWebrtc():
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
        print("Robot Signal thread start!")
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout = 0.1) #block 아님?
            
                # item is a dict, so use key access instead of attributes
                if item.get('payload_type') == None:
                    print("payload_type is None")
                    self.queue.task_done()
                    continue
                
                print(f'get Item!{item.get("payload_type")}')
                
                self.queue.task_done()
            except queue.Empty:
                pass
