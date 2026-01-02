import grpc_commander.grpc.generated.robot_request_signal_pb2 as signal_pb
import grpc_commander.grpc.generated.robot_request_signal_pb2_grpc as signal_pb2_grpc

import threading
import time
import queue
from google.protobuf.json_format import MessageToDict

class RobotRequestSignalService:
   
    def __init__(self, signal_stub: signal_pb2_grpc.RobotSignalServiceStub, signal_queue: queue.Queue, robot_id, heartbeat_interval: float = 1.0):
        self.signal_stub = signal_stub
        self.signal_queue = signal_queue  # incoming messages to be consumed by WebRTC handler
        self.robot_id = robot_id
        self.heartbeat_interval = heartbeat_interval

        self.stop_event = threading.Event()
        self.outgoing_queue: queue.Queue = queue.Queue() # for the purpose of sending message to server
        self.worker = threading.Thread(target=self._run_stream, daemon=True)

    def __del__(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1)

    def run(self):
        if not self.worker.is_alive():
            self.worker.start()

    def send_signal(self, message: signal_pb.SignalMessage):
        self.outgoing_queue.put(message)

    def _request_iterator(self):
        # Initial stream creation / data request
        print("request start")
        yield signal_pb.SignalMessage(
            robot_id=self.robot_id,
            screen_request=signal_pb.ScreenRequest(),
        )
        print("request end")
        next_heartbeat = time.time() + self.heartbeat_interval

        while not self.stop_event.is_set():
            timeout = max(0, next_heartbeat - time.time())
            try:
                message = self.outgoing_queue.get(timeout=timeout)
                if message is None:
                    continue
                yield message
                next_heartbeat = time.time() + self.heartbeat_interval
            except queue.Empty:
                print("heartbeat start")
                yield signal_pb.SignalMessage(robot_id=self.robot_id) #work here?
                print("heartbeat end")
                next_heartbeat = time.time() + self.heartbeat_interval
            

    def _run_stream(self):
        print("Robot Signal Service thread start!")
        try:
            response_stream = self.signal_stub.OpenSignalStream(self._request_iterator())
            for response in response_stream:
                print("run?")
                if self.stop_event.is_set():
                    print("run end")
                    break
                print("keep listening")
                self._handle_response(response)
        except Exception as e:
            print(f"Signal stream error: {str(e)}")

    def _handle_response(self, response: signal_pb.SignalMessage):
        payload_type = response.WhichOneof("payload")
        payload = {
            "robot_id": response.robot_id,
            "payload_type": payload_type,
        }

        if payload_type:
            payload[payload_type] = MessageToDict(
                getattr(response, payload_type),
                preserving_proto_field_name=True,
            )

        self.signal_queue.put(payload)
