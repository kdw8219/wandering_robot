import os
import queue
import threading
from typing import Optional, Callable, Dict, Any
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    MediaStreamTrack,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.sdp import candidate_from_sdp
from aiortc.contrib.media import MediaPlayer
import asyncio

class MediaSource:
    def __init__(self, frame_rate: int = 30, frame_size = "1280x720"):
        self.frame_rate = frame_rate
        self.frame_size = frame_size
        self.player: Optional[MediaPlayer] = None
        
        
    def start(self) -> MediaStreamTrack:
        self.player = MediaPlayer(
            file=":0",
            format="x11grab",
            options={
                "framerate": str(self.frame_rate),
                "video_size": self.frame_size,
            }
        )
        return self.player.video

    def stop(self) -> None:
        if self.player:
            self.player.stop()
            self.player = None
        pass


class WebrtcSession:
    def __init__(
        self,
        media_source: MediaSource,
        on_ice_candidate: Optional[Callable[[Dict[str, Any]], None]] = None,
        rtc_configuration: Optional[RTCConfiguration] = None,
    ):
        self.media_source = media_source
        self.pc: Optional[RTCPeerConnection] = None
        self.active = False
        self.on_ice_candidate = on_ice_candidate
        self.rtc_configuration = rtc_configuration

    def start(self) -> None:
        if self.active:
            return
        self.pc = RTCPeerConnection(self.rtc_configuration)
        if self.on_ice_candidate:
            @self.pc.on("icecandidate")
            async def _on_icecandidate(candidate):
                if candidate is None:
                    print("ICE gathering complete but no candidate found.")
                    return
                print(f"Local ICE candidate: {candidate.candidate}")
                self.on_ice_candidate(
                    {
                        "candidate": candidate.candidate,
                        "sdp_mid": candidate.sdpMid,
                        "sdp_mline_index": candidate.sdpMLineIndex,
                    }
                )
        @self.pc.on("icegatheringstatechange")
        async def _on_icegatheringstatechange():
            print(f"ICE gathering state: {self.pc.iceGatheringState}")
        @self.pc.on("iceconnectionstatechange")
        async def _on_iceconnectionstatechange():
            print(f"ICE connection state: {self.pc.iceConnectionState}")
        media_stream_track = self.media_source.start()
        if media_stream_track:
            self.pc.addTrack(media_stream_track)
        self.active = True

    def stop(self) -> None:
        if not self.active:
            return
        self.media_source.stop()
        
        if self.pc:
            # self.pc.close()가 async이기 때문에 이벤트 루프에서 close해서 완전 처리 필요
            asyncio.get_event_loop().run_until_complete(self.pc.close())
            self.pc = None
        
        self.active = False

    async def _create_offer(self):
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        return self.pc.localDescription.sdp

    def create_offer(self) -> Optional[str]:
        if self.active is False or self.pc is None:
            return None
        
        result = asyncio.get_event_loop().run_until_complete(self._create_offer())
        return result

    async def _apply(self, sdp):
        desc = RTCSessionDescription(sdp=sdp, type="answer")
        await self.pc.setRemoteDescription(desc)
        
    def handle_answer(self, sdp: str) -> None:
        if self.active is False and self.pc is None:
            return None
        
        asyncio.get_event_loop().run_until_complete(self._apply(sdp))

    async def _add(self, candidate):
        ice = candidate_from_sdp(candidate.get("candidate", ""))
        ice.sdpMid = candidate.get("sdp_mid")
        ice.sdpMLineIndex = candidate.get("sdp_mline_index")
        await self.pc.addIceCandidate(ice)


    def add_ice_candidate(self, candidate: dict) -> None:
        
        if self.active is False or self.pc is None:
            return None
        
        asyncio.get_event_loop().run_until_complete(self._add(candidate))


class RobotWebrtc:
    def __init__(self, queue: queue.Queue, response_queue: queue.Queue):
        self.queue = queue
        self.response_queue = response_queue
        self.worker = threading.Thread(target=self.working_thread, daemon=True)
        self.stop_event = threading.Event()
        self.rtc_configuration = self._build_rtc_configuration()
        self.webrtc_session = WebrtcSession(
            MediaSource(), self._handle_local_ice, self.rtc_configuration
        )
        self.client_ip_override = ""
        self.wsl_host_ip = ""
        

    def _build_rtc_configuration(self) -> Optional[RTCConfiguration]:
        urls_env = os.environ.get("WEBRTC_TURN_URLS", "")
        if not urls_env:
            return None
        urls = [url.strip() for url in urls_env.split(",") if url.strip()]
        if not urls:
            return None
        ice_server_kwargs: Dict[str, Any] = {"urls": urls}
        username = os.environ.get("WEBRTC_TURN_USERNAME", "")
        credential = os.environ.get("WEBRTC_TURN_CREDENTIAL", "")
        
        print(f"TURN server configured with URLs: {urls}")
        print(f"TURN username: {username}")
        print(f"TURN credential: {'***' if credential else ''}")
        if username:
            ice_server_kwargs["username"] = username
        if credential:
            ice_server_kwargs["credential"] = credential
        return RTCConfiguration(iceServers=[RTCIceServer(**ice_server_kwargs)])

    def _handle_local_ice(self, candidate: Dict[str, Any]) -> None:
        self.response_queue.put(
            {
                "type": "robot_ice",
                "candidate": candidate.get("candidate", ""),
                "sdp_mid": candidate.get("sdp_mid", ""),
                "sdp_mline_index": candidate.get("sdp_mline_index", 0),
            }
        )

    def __del__(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1)

    def run(self) -> None:
        if not self.worker.is_alive():
            self.worker.start()

    def working_thread(self) -> None:
        print("Robot Signal thread start!")
        asyncio.set_event_loop(asyncio.new_event_loop())
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout=0.1)
                payload_type = item.get("payload_type")
                if payload_type is None:
                    self.queue.task_done()
                    continue

                print(f'get Item!{payload_type}')
                self._handle_payload(payload_type, item)
                self.queue.task_done()
            except queue.Empty:
                pass

    def _handle_payload(self, payload_type: str, item: dict) -> None:
        if payload_type == "screen_request":
            self.webrtc_session.start()
            offer_sdp = self.webrtc_session.create_offer()
            if offer_sdp is None:
                return
            self.response_queue.put(
                {
                    "type": "robot_offer",
                    "sdp": offer_sdp,
                }
            )
            return
 
        if payload_type == "client_answer":
            client_answer = item.get("client_answer", {})
            answer = client_answer.get("sdp")
            if answer:
                self.webrtc_session.handle_answer(answer)
            return
            
        if payload_type == "client_ice":
            candidate = item.get("client_ice")
            if candidate:
                self.webrtc_session.add_ice_candidate(
                    self._rewrite_client_candidate(candidate)
                )
            return

    def _rewrite_client_candidate(self, candidate: dict) -> dict:
        print('work?')
        candidate_sdp = candidate.get("candidate", "")
        if not self.wsl_host_ip or not candidate_sdp:
            return candidate
        parts = candidate_sdp.split()
        if len(parts) < 6:
            return candidate
        address = parts[4]
        if self.client_ip_override and address == self.client_ip_override:
            parts[4] = self.wsl_host_ip
        elif address.endswith(".local"):
            parts[4] = self.wsl_host_ip
        else:
            return candidate
        updated = dict(candidate)
        updated["candidate"] = " ".join(parts)
        print(f"Rewriting candidate: {candidate_sdp} -> {updated['candidate']}")
        return updated
