#!/usr/bin/env python3
"""
Minimal IMU Data Reader
Prints IMU data from iPhone to stdout
"""

import asyncio
import sys
import os

# Add arvos-sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))

from arvos import ArvosServer, IMUData


async def main():
    # Create server on port 8765 (same as your web viewer)
    server = ArvosServer(host="0.0.0.0", port=8765)
    
    
    
    # Debug: print ALL messages received
    async def on_message(client_id: str, message):
        print(f"📨 RAW MESSAGE from {client_id}: {message[:200] if isinstance(message, (str, bytes)) else message}")
    
    # Handle IMU data - print everything (regular def, not async)
    def on_imu(data: IMUData):
        print(f"📊 IMU: accel=({data.linear_acceleration[0]:.3f}, {data.linear_acceleration[1]:.3f}, {data.linear_acceleration[2]:.3f}) "
              f"gyro=({data.angular_velocity[0]:.3f}, {data.angular_velocity[1]:.3f}, {data.angular_velocity[2]:.3f}) "
              f"timestamp={data.timestamp_ns/1e9:.3f}")
    
    # Connection status
    async def on_connect(client_id: str):
        print(f"✅ iPhone connected: {client_id}")
        print("Waiting for IMU data...")
    
    async def on_disconnect(client_id: str):
        print(f"❌ iPhone disconnected: {client_id}")
    
    # Register handlers
    server.on_message = on_message
    server.on_imu = on_imu
    server.on_connect = on_connect
    server.on_disconnect = on_disconnect
    
    # Start server
    print("🚀 Starting IMU server on port 8765...")
    print("Connect your iPhone to this server and start streaming.\n")
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 Stopped")
