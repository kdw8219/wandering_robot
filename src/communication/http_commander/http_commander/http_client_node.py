import rclpy
from rclpy.node import Node
import aiohttp
import asyncio
import threading
import jwt
import json

class HttpClientNode(Node):

    def __init__(self):
        super().__init__('http_client_node')

        # PARAMETERS
        self.declare_parameter("server_url", "http://127.0.0.1")
        self.declare_parameter("server_port", 80)
        self.declare_parameter("robot_id", "abcd")
        self.declare_parameter("robot_secret", "abcdzxcv")
        self.declare_parameter("request_expired", 3.0)
        self.declare_parameter("access_secret_key", "abcdefg")

        self.url = self.get_parameter("server_url").value
        self.port = str(self.get_parameter("server_port").value)
        self.robot_id = self.get_parameter("robot_id").value
        self.robot_secret = self.get_parameter("robot_secret").value
        self.request_expired = self.get_parameter("request_expired").value
        self.access_secret_key = self.get_parameter("access_secret_key").value

        # TOKENS
        self.on_refreshing = False
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
        self.timer = self.create_timer(3, self.timer_callback)
        
        # JWT Handler
        #self.jwt_timer = self.create_timer(5, self.)

        self.get_logger().info("HTTP Client Node Started.")


    # =======================================================
    # Event Loop Runner (thread)
    # =======================================================
    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def heartbeat(self):
        url = f"{self.url}:{self.port}/api/robots/heartbeat/"
        self.get_logger().info(f"Trying refresh: {url}")
        payload = {
            'robot_id': self.robot_id
        }

        try:
            timeout = aiohttp.ClientTimeout(total=self.request_expired)

            async with self.client.post(url, timeout=timeout, json=payload) as resp:
                resp.raise_for_status()
                text = await resp.text()

            self.get_logger().info(f"Heartbeating Success. + {text}")

        except asyncio.TimeoutError:
            self.get_logger().error("Heartbeating timed out!")
        except aiohttp.ClientResponseError as e:
            self.get_logger().error(f"HTTP Client error: {e}")
        except aiohttp.ClientError as e:
            self.get_logger().error(f"HTTP Client error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        
        return

    # =======================================================
    # refresh access token using refresh token (thread)
    # =======================================================
    async def refresh_token(self):
        
        url = f"{self.url}:{self.port}/api/robots/tokens/refresh/"
        self.get_logger().info(f"Trying refresh: {url}")
        payload = {
            'refresh_token': self.refresh_token
        }

        try:
            timeout = aiohttp.ClientTimeout(total=self.request_expired)

            async with self.client.post(url, timeout=timeout, json=payload) as resp:
                resp.raise_for_status()
                text = await resp.text()
                data_json = json.loads(text)
                
                self.access_token = data_json['access_token']
                
                self.on_refreshing=False

            self.get_logger().info(f"Token Refresh Success.")

        except asyncio.TimeoutError:
            self.get_logger().error("Token Refresh timed out!")
        except aiohttp.ClientResponseError as e:
            if e.status == 401: #Unauthorized
                self.login_tried = False
                self.get_logger().error(f"Login Expired: {e}")
            else:
                self.get_logger().error(f"HTTP Client error: {e}")
        except aiohttp.ClientError as e:
            self.get_logger().error(f"HTTP Client error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        
        return
    
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
        
        #Not Connected
        if self.client is None or self.login_tried is None:
            return
        
        #Connected
        #token process, if access token is expired
        # if self.access_token != "" and self.valid_access_token() is False:
        #     self.on_refreshing = True
        #     asyncio.run_coroutine_threadsafe(self.refresh_token(), self.loop)
        
        #heartbeat
        #send heartbeat
        #just of heartbeating. using robot_id
        
        asyncio.run_coroutine_threadsafe(self.heartbeat(), self.loop)
        
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

            self.get_logger().info(f"Login success + {text}.")
            self.login_tried = True
            
            json_data = json.loads(text)
            self.access_token = json_data['access_token']
            self.refresh_token = json_data['refresh_token']
            
        except asyncio.TimeoutError:
            self.get_logger().error("Login request timed out!")
        except aiohttp.ClientError as e:
            self.get_logger().error(f"HTTP Client error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON. Error: {e}")
            self.get_logger().error(f"Problematic text content: {text}")


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
