#!/usr/bin/env python3
"""
Live IMU Visualization
Shows real-time plots of accelerometer and gyroscope data
"""

import asyncio
import sys
import os
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Add arvos-sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))

from arvos import ArvosServer, IMUData


class IMUVisualizer:
    def __init__(self, max_points=500):
        self.max_points = max_points
        
        # Data buffers
        self.timestamps = deque(maxlen=max_points)
        self.accel_x = deque(maxlen=max_points)
        self.accel_y = deque(maxlen=max_points)
        self.accel_z = deque(maxlen=max_points)
        self.gyro_x = deque(maxlen=max_points)
        self.gyro_y = deque(maxlen=max_points)
        self.gyro_z = deque(maxlen=max_points)
        
        self.start_time = None
        self.data_count = 0
        
        # Setup plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('Live IMU Data - Skateboard Sensors', fontsize=16, fontweight='bold')
        
        # Accelerometer plot
        self.line_ax, = self.ax1.plot([], [], 'r-', label='Accel X', linewidth=2)
        self.line_ay, = self.ax1.plot([], [], 'g-', label='Accel Y', linewidth=2)
        self.line_az, = self.ax1.plot([], [], 'b-', label='Accel Z', linewidth=2)
        self.ax1.set_ylabel('Acceleration (m/s²)', fontsize=12)
        self.ax1.set_title('Accelerometer', fontweight='bold')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(-20, 20)
        
        # Gyroscope plot
        self.line_gx, = self.ax2.plot([], [], 'r-', label='Gyro X', linewidth=2)
        self.line_gy, = self.ax2.plot([], [], 'g-', label='Gyro Y', linewidth=2)
        self.line_gz, = self.ax2.plot([], [], 'b-', label='Gyro Z', linewidth=2)
        self.ax2.set_xlabel('Time (seconds)', fontsize=12)
        self.ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
        self.ax2.set_title('Gyroscope', fontweight='bold')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_ylim(-5, 5)
        
        plt.tight_layout()
    
    def add_data(self, data: IMUData):
        """Add new IMU data point"""
        if self.start_time is None:
            self.start_time = data.timestamp_ns
        
        # Convert timestamp to seconds from start
        t = (data.timestamp_ns - self.start_time) / 1e9
        
        self.timestamps.append(t)
        self.accel_x.append(data.linear_acceleration[0])
        self.accel_y.append(data.linear_acceleration[1])
        self.accel_z.append(data.linear_acceleration[2])
        self.gyro_x.append(data.angular_velocity[0])
        self.gyro_y.append(data.angular_velocity[1])
        self.gyro_z.append(data.angular_velocity[2])
        
        self.data_count += 1
    
    def update_plot(self, frame):
        """Update the plot with new data"""
        if len(self.timestamps) == 0:
            return (self.line_ax, self.line_ay, self.line_az, 
                    self.line_gx, self.line_gy, self.line_gz)
        
        t = np.array(self.timestamps)
        
        # Update accelerometer
        self.line_ax.set_data(t, np.array(self.accel_x))
        self.line_ay.set_data(t, np.array(self.accel_y))
        self.line_az.set_data(t, np.array(self.accel_z))
        
        # Update gyroscope
        self.line_gx.set_data(t, np.array(self.gyro_x))
        self.line_gy.set_data(t, np.array(self.gyro_y))
        self.line_gz.set_data(t, np.array(self.gyro_z))
        
        # Update x-axis limits (show last 5 seconds)
        if len(t) > 0:
            x_max = t[-1]
            x_min = max(0, x_max - 5)
            self.ax1.set_xlim(x_min, x_max)
            self.ax2.set_xlim(x_min, x_max)
        
        # Update title with data rate
        if len(t) > 1:
            dt = t[-1] - t[0]
            rate = len(t) / dt if dt > 0 else 0
            self.fig.suptitle(
                f'Live IMU Data - Skateboard Sensors | Rate: {rate:.1f} Hz | Samples: {self.data_count}',
                fontsize=16, fontweight='bold'
            )
        
        return (self.line_ax, self.line_ay, self.line_az, 
                self.line_gx, self.line_gy, self.line_gz)


# Global visualizer instance
viz = IMUVisualizer()


def run_server():
    """Run asyncio server in a separate thread"""
    async def server_main():
        # Create server
        server = ArvosServer(host="0.0.0.0", port=8765)
        
        # Print QR code
        server.print_qr_code()
        
        # Handle IMU data
        def on_imu(data: IMUData):
            viz.add_data(data)
        
        # Connection status
        async def on_connect(client_id: str):
            print(f"✅ iPhone connected: {client_id}")
            print("📊 Streaming data to live plot...")
        
        async def on_disconnect(client_id: str):
            print(f"❌ iPhone disconnected: {client_id}")
        
        # Register handlers
        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        
        # Start server
        print("🚀 Starting IMU server on port 8765...")
        print("📈 Live plot will open shortly...\n")
        
        await server.start()
    
    # Run the async server
    asyncio.run(server_main())


if __name__ == "__main__":
    try:
        # Start server in background thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        # Give server time to start
        import time
        time.sleep(1)
        
        # Start animation in main thread
        ani = FuncAnimation(viz.fig, viz.update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.show()
        
    except KeyboardInterrupt:
        print("\n\n👋 Stopped")
