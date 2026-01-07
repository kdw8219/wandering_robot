import queue
import threading
from typing import Optional
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, MediaStreamTrack
from aiortc.contrib.media import MediaPlayer

class MediaSource:
    def __init__(self, frame_rate: int = 30, frame_size = "1280x720"):
        self.frame_rate = frame_rate,
        self.frame_size = frame_size,
        self.player:Optional[MediaPlayer] = None,
        
        
    def start(self) -> None:
        self.player = MediaPlayer(
            format="x11grab",
            options={
                "framerate": str(self.frame_rate),
                "video_size": self.frame_size,
            }
        )
        return self.player.video

    def stop(self) -> None:
        self.player.stop()
        self.player = None
        pass


class WebrtcSession:
    def __init__(self, media_source: MediaSource):
        self.media_source = media_source
        self.active = False

    def start(self) -> None:
        if self.active:
            return
        self.media_source.start()
        self.active = True

    def stop(self) -> None:
        if not self.active:
            return
        self.media_source.stop()
        self.active = False

    def create_offer(self) -> Optional[str]:
        if not self.active:
            return None
        # TODO: create SDP offer from PeerConnection.
        return None

    def handle_answer(self, sdp: str) -> None:
        # TODO: apply remote SDP answer to PeerConnection.
        pass

    def add_ice_candidate(self, candidate: dict) -> None:
        # TODO: add remote ICE candidate to PeerConnection.
        pass


class RobotWebrtc:
    """Coordinates signaling payloads with a WebRTC session."""

    def __init__(self, queue: queue.Queue, session: WebrtcSession):
        self.queue = queue
        self.session = session
        self.worker = threading.Thread(target=self.working_thread, daemon=True)
        self.stop_event = threading.Event()

    def __del__(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1)

    def run(self) -> None:
        if not self.worker.is_alive():
            self.worker.start()

    def working_thread(self) -> None:
        print("Robot Signal thread start!")
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout=0.1)
                payload_type = item.get("payload_type")
                if payload_type is None:
                    print("payload_type is None")
                    self.queue.task_done()
                    continue

                print(f'get Item!{payload_type}')
                self._handle_payload(payload_type, item)
                self.queue.task_done()
            except queue.Empty:
                pass

    def _handle_payload(self, payload_type: str, item: dict) -> None:
        if payload_type == "screen_request":
            self.session.start()
            # TODO: Robot offer 만들기 전에 영상 관련 처리 끝내놓고 RobotOffer 전송 필요.
            # TODO: 일단 js 보고 초안은 잡아놨는데 나머지는 내일...
            return
        if payload_type == "client_answer":
            answer = item.get("client_answer", {}).get("sdp")
            if answer:
                self.session.handle_answer(answer)
            return
        if payload_type == "client_ice":
            candidate = item.get("client_ice")
            if candidate:
                self.session.add_ice_candidate(candidate)
            return
