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
        self.worker.run()
        
    def workingThread(self):
        
        while not self.stop_event.is_set():
            item = self.queue.get()
            
            print(f'get Item!{item.command}')
            
            self.queue.task_done()