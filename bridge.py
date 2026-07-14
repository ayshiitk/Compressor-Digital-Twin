import asyncio
import websockets
import json
import threading
import queue

class TelemetryBridge:
    def __init__(self, host="localhost", port=8765):
        self.host = host
        self.port = port
        self._clients = set()
        self._cmd_queue = queue.Queue()
        self._loop = None
        self._thread = None
        self._running = False
        self._stop_event = None

    def start(self):
        """Starts the WebSocket server in a daemon thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        print(f"[TelemetryBridge] Started WebSocket server on ws://{self.host}:{self.port}")

    def stop(self):
        """Stops the WebSocket server safely."""
        self._running = False
        if self._loop and self._stop_event:
            # Tell the async loop to stop blocking and shut down the server
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread:
            self._thread.join(timeout=2)
        print("[TelemetryBridge] Stopped.")

    def broadcast(self, state_dict):
        """Call this from your sim loop to push the latest state to all clients."""
        if not self._clients or not self._loop:
            return
        
        # Serialize once
        msg = json.dumps(state_dict)
        
        # Schedule the broadcast on the async event loop
        asyncio.run_coroutine_threadsafe(self._broadcast_async(msg), self._loop)

    def get_commands(self):
        """Call this in your sim loop to get a list of all pending commands from the dashboard."""
        cmds = []
        while not self._cmd_queue.empty():
            try:
                cmds.append(self._cmd_queue.get_nowait())
            except queue.Empty:
                break
        return cmds

    async def _broadcast_async(self, msg):
        if not self._clients:
            return
        # Await gathering of sends to all connected clients
        await asyncio.gather(
            *[client.send(msg) for client in self._clients],
            return_exceptions=True
        )

    async def _handler(self, websocket):
        """Handles incoming connections and messages."""
        self._clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self._cmd_queue.put(data)
                except json.JSONDecodeError:
                    print(f"[TelemetryBridge] Received invalid JSON: {message}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.remove(websocket)

    def _run_server(self):
        """The core thread runner."""
        # Create a new event loop for this background thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        # Wrapper to run the server INSIDE the active loop
        async def run_async_server():
            self._stop_event = asyncio.Event()
            
            # Start the websocket server
            async with websockets.serve(self._handler, self.host, self.port):
                # Keep the server running until stop() sets this event
                await self._stop_event.wait()
                
        # Run the wrapper
        self._loop.run_until_complete(run_async_server())