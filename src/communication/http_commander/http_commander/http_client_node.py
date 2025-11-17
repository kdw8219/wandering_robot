import rclpy
from rclpy.node import Node
import aiohttp
import asyncio
import threading


class HttpClientNode(Node):

    def __init__(self):
        super().__init__('http_client_node')

        # PARAMETERS
        self.declare_parameter("server_url", "http://127.0.0.1")
        self.declare_parameter("server_port", 80)
        self.declare_parameter("robot_id", "abcd")
        self.declare_parameter("robot_secret", "abcdzxcv")
        self.declare_parameter("request_expired", 3.0)

        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.robot_id = self.get_parameter("robot_id").value
        self.robot_secret = self.get_parameter("robot_secret").value
        self.request_expired = self.get_parameter("request_expired").value

        # TOKENS
        self.access_token = ""
        self.refresh_token = ""

        # Async components
        self.loop = asyncio.new_event_loop()
        self.client: aiohttp.ClientSession | None = None
        self.login_tried = False

        # Run event loop in background thread
        self.loop_thread = threading.Thread(target=self.start_loop, daemon=True)
        self.loop_thread.start()

        # ROS timer
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # JWT Handler
        #self.jwt_timer = self.create_timer(5, self.)

        self.get_logger().info("HTTP Client Node Started.")


    # =======================================================
    # Event Loop Runner (thread)
    # =======================================================
    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()


    # =======================================================
    # Timer callback
    # =======================================================
    def timer_callback(self):
        if self.client is None:
            asyncio.run_coroutine_threadsafe(self.initialize_session(), self.loop)
            return

        if not self.login_tried:
            asyncio.run_coroutine_threadsafe(self.try_login_once(), self.loop)
            return
        
        #run additional (like jwt)
        
        if self.client is not None and self.login_tried is not None:
            pass


    # =======================================================
    # Create aiohttp session
    # =======================================================
    async def initialize_session(self):
        if self.client is None:
            try:
                self.client = aiohttp.ClientSession()
                self.get_logger().info("ClientSession initialized.")
            except Exception as e:
                self.get_logger().error(f"ClientSession init failed: {e}")


    # =======================================================
    # Login Once
    # =======================================================
    async def try_login_once(self):
        if self.login_tried or self.client is None:
            return

        url = f"{self.url}:{self.port}/api/robots/login/"
        self.get_logger().info(f"Trying login: {url}")
        payload = {
            'robot_id': self.robot_id,
            'robot_secret': self.robot_secret
        }

        try:
            timeout = aiohttp.ClientTimeout(total=self.request_expired)

            async with self.client.post(url, timeout=timeout, json=payload) as resp:
                resp.raise_for_status()
                text = await resp.text()

            self.get_logger().info(f"Login success ({len(text)} chars).")
            self.login_tried = True

        except asyncio.TimeoutError:
            self.get_logger().error("Login request timed out!")
        except aiohttp.ClientError as e:
            self.get_logger().error(f"HTTP Client error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


    # =======================================================
    # Check JWT Status
    # =======================================================


    # =======================================================
    # Shutdown (node destroy)
    # =======================================================
    async def async_shutdown(self):
        if self.client:
            await self.client.close()
            self.get_logger().info("HTTP Session Closed.")
        self.loop.stop()

    def destroy_node(self):
        future = asyncio.run_coroutine_threadsafe(self.async_shutdown(), self.loop)
        future.result()
        super().destroy_node()


# ================================================================
# MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = HttpClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
