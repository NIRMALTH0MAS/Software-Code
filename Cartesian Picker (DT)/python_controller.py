#!/usr/bin/env python3
"""
Python WebSocket Server for Cartesian Picker Control
Sends position commands to the web interface in the format: {x: value, y: value, z: value}
"""

import asyncio
import websockets
import json
import time

class CartesianPickerController:
    def __init__(self, host='localhost', port=8765):
        self.host = host
        self.port = port
        self.clients = set()

    async def register(self, websocket):
        """Register a new client connection"""
        self.clients.add(websocket)
        print(f"Client connected. Total clients: {len(self.clients)}")

    async def unregister(self, websocket):
        """Unregister a client connection"""
        self.clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(self.clients)}")

    async def send_position(self, x, y, z):
        """Send position command to all connected clients"""
        if not self.clients:
            print("No clients connected")
            return

        message = json.dumps({"x": x, "y": y, "z": z})
        
        # Send to all connected clients
        disconnected_clients = set()
        for client in self.clients:
            try:
                await client.send(message)
                print(f"Sent position: x={x}, y={y}, z={z}")
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)

        # Remove disconnected clients
        for client in disconnected_clients:
            await self.unregister(client)

    async def handler(self, websocket, *args, **kwargs):
        """Handle WebSocket connections (robust signature for all library versions)"""
        await self.register(websocket)
        try:
            # Keep connection alive and listen for messages
            async for message in websocket:
                # Echo received messages (optional)
                print(f"Received: {message}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister(websocket)

    async def start_server(self):
        """Start the WebSocket server"""
        async with websockets.serve(self.handler, self.host, self.port):
            print(f"WebSocket server started on ws://{self.host}:{self.port}")
            print("Waiting for connections...")
            await asyncio.Future()  # Run forever

    def run(self):
        """Run the server"""
        asyncio.run(self.start_server())


async def example_control_sequence(controller):
    """Example: Send a sequence of position commands"""
    print("\nStarting example control sequence...")
    print("Make sure the web interface is open and connected!\n")
    
    await asyncio.sleep(2)  # Wait for connection
    
    # Move to different positions (0-100 scale maps to physical limits)
    # X: -3.1 to -0.5 | Y: -8.9 to 0 | Z: -2 to 1
    positions = [
        {"x": 50, "y": 0, "z": 0},      # Move X to midpoint
        {"x": 50, "y": 100, "z": 0},    # Move Y to max (0)
        {"x": 50, "y": 100, "z": 100},  # Move Z to max (1)
        {"x": 0, "y": 100, "z": 100},   # Move X to min (-3.1)
        {"x": 0, "y": 0, "z": 100},     # Move Y to min (-8.9)
        {"x": 0, "y": 0, "z": 0},       # Return all to min
        {"x": 50, "y": 50, "z": 50},    # Return all to midpoints
    ]
    
    for pos in positions:
        # You can also pass "mode": "physical" to use real coordinates
        # e.g. {"x": -1.5, "y": -4.0, "z": 0.0, "mode": "physical"}
        await controller.send_position(pos.get("x", 0), pos.get("y", 0), pos.get("z", 0))
        await asyncio.sleep(6)  # Wait for 5-second animation + buffer


async def main():
    """Main function to run server and example"""
    controller = CartesianPickerController()
    
    # Start server in background
    server_task = asyncio.create_task(controller.start_server())
    
    # Wait a bit for server to start
    await asyncio.sleep(1)
    
    # Run example sequence
    await example_control_sequence(controller)
    
    # Keep server running
    await server_task


if __name__ == "__main__":
    print("=" * 60)
    print("Cartesian Picker WebSocket Controller")
    print("=" * 60)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
