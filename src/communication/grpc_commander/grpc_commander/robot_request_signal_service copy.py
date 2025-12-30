import threading
import time
import queue
from typing import Callable, Iterable, Optional, Union
from google.protobuf.json_format import MessageToDict

import grpc_commander.grpc.generated.robot_request_signal_pb2 as signal_pb
import grpc_commander.grpc.generated.robot_request_signal_pb2_grpc as signal_pb2_grpc


class RobotRequestSignalService:
    """
    Keep a bidirectional Signal stream open from the robot to the server.
    - Send an initial stream creation request.
    - Send heartbeat messages periodically.
    - Block waiting for data from the server and forward to the consumer queue.
    """

    def __init__(
        self,
        signal_stub: signal_pb2_grpc.RobotSignalServiceStub,
        signal_queue: queue.Queue,
        robot_id,
        heartbeat_interval: float = 5.0,
        response_handler: Optional[Callable[[dict], Union[None, signal_pb.SignalMessage, Iterable[signal_pb.SignalMessage]]]] = None,
    ):
        self.signal_stub = signal_stub
        self.signal_queue = signal_queue  # incoming messages to be consumed by WebRTC handler
        self.robot_id = robot_id
        self.heartbeat_interval = heartbeat_interval
        self.response_handler = response_handler  # 서버 응답을 받아 바로 응답을 생성하고 싶을 때 사용

        self.stop_event = threading.Event()
        self.outgoing_queue: queue.Queue = queue.Queue()
        self.worker = threading.Thread(target=self._run_stream, daemon=True)

    def __del__(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1)

    def run(self):
        if not self.worker.is_alive():
            self.worker.start()

    def send_signal(self, message: signal_pb.SignalMessage):
        """Queue an outgoing SignalMessage to send over the open stream."""
        self.outgoing_queue.put(message)

    def _request_iterator(self):
        # Initial stream creation / data request
        yield signal_pb.SignalMessage(
            robot_id=self.robot_id,
            screen_request=signal_pb.ScreenRequest(),
        )

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
                # Heartbeat has priority when no other message is waiting.
                yield signal_pb.SignalMessage(robot_id=self.robot_id)
                next_heartbeat = time.time() + self.heartbeat_interval

    def _run_stream(self):
        print("Robot Signal Service thread start!")
        try:
            response_stream = self.signal_stub.OpenSignalStream(self._request_iterator())
            for response in response_stream:
                if self.stop_event.is_set():
                    break
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

        # 서버 응답을 즉시 처리하고, 필요 시 결과를 동일 스트림으로 보낼 수 있는 훅
        if self.response_handler:
            try:
                reply = self.response_handler(payload)
                if reply is None:
                    return

                # 단일 메시지나 반복 가능한 메시지 모두 허용
                if isinstance(reply, signal_pb.SignalMessage):
                    self.send_signal(reply)
                else:
                    for item in reply:
                        if isinstance(item, signal_pb.SignalMessage):
                            self.send_signal(item)
            except Exception as e:
                # 응답 처리에서 에러가 나도 스트림은 계속 유지
                print(f"Signal response handler error: {str(e)}")
