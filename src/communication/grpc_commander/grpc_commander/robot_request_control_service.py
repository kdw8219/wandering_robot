import grpc_commander.grpc.generated.robot_request_control_pb2 as control_pb
import grpc_commander.grpc.generated.robot_request_control_pb2_grpc as control_pb2_grpc
import threading
import queue
from google.protobuf.json_format import MessageToDict
import json

class RobotRequestControlService():

    def __init__(self, control_stub: control_pb2_grpc.RobotRequestControlServiceStub, queue:queue.Queue, robot_id):
        self.is_stop = False
        self.queue = queue
        self.thread = threading.Thread(target=self.execute_command_async)
        self.stop_event = threading.Event()
        self.control_stub = control_stub
        self.robot_id = robot_id
        
    def __del__(self):
        self.stop_event.set()
        self.thread.join()
        
    def run(self):
        self.thread.start()

    def execute_command_async(self):
        print("Robot Control Service thread start!")
        while not self.stop_event.is_set():
            try:
                req = control_pb.RobotCommandRequest(
                    robot_id=self.robot_id
                )
                resp = self.control_stub.GetNextCommand(req) 
                payload = {
                        "has_command": resp.has_command,
                        "command": control_pb.CommandType.Name(resp.command),
                }
                
                if resp.has_command == False:
                    continue
                else:
                    payload_type = resp.WhichOneof("payload")

                    print(f"Received command: {control_pb.CommandType.Name(resp.command)}")

                    if payload_type is not None:
                        payload[payload_type] = MessageToDict(
                            getattr(resp, payload_type),
                            preserving_proto_field_name=True,
                        )
                
                
                self.queue.put(json.dumps(payload))
            
            except Exception as e:
                print(f'Error...:{str(e)}') #어쨌든 queue 터지지 않게 소모는 시켜줘야 할듯
