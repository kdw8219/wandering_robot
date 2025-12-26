import grpc_commander.grpc.generated.robot_request_signal_pb2 as signal_pb
import grpc_commander.grpc.generated.robot_request_signal_pb2_grpc as signal_pb2_grpc
import threading
import queue
from google.protobuf.json_format import MessageToDict
import json

class RobotRequestSignalService():

    def __init__(self, signal_stub: signal_pb2_grpc.RobotSignalServiceStub, queue:queue.Queue, robot_id):
        self.is_stop = False
        self.queue = queue
        self.thread = threading.Thread(target=self.execute_command_async)
        self.stop_event = threading.Event()
        self.signal_stub = signal_stub
        self.robot_id = robot_id
        
    def __del__(self):
        self.stop_event.set()
        self.thread.join()
        
    def run(self):
        self.thread.start()

    def execute_command_async(self):
        print("Robot Signal Service thread start!")
        while not self.stop_event.is_set():
            try:
                req = signal_pb.RobotCommandRequest(
                    robot_id=self.robot_id
                )
                resp = self.control_stub.GetNextCommand(req) 
                payload = {
                        "has_command": resp.has_command,
                        "command": signal_pb.CommandType.Name(resp.command),
                }
                
                if resp.has_command == False:
                    continue
                else:
                    payload_type = resp.WhichOneof("payload")
            
                    
                    if payload_type == "move":
                        payload[payload_type] = MessageToDict(resp.move, preserving_proto_field_name=True)
                    elif payload_type == "set_speed":
                        payload[payload_type] = MessageToDict(resp.set_speed, preserving_proto_field_name=True)
                    elif payload_type == "path_follow":
                        payload[payload_type] = MessageToDict(resp.path_follow, preserving_proto_field_name=True)
                
                
                self.queue.put(json.dumps(payload))
            
            except Exception as e:
                print(f'Error...:{str(e)}') #어쨌든 queue 터지지 않게 소모는 시켜줘야 할듯
