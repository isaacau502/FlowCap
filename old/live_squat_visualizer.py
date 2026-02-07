#!/usr/bin/env python3
"""
Live Squat & Torso Rotation Visualizer
Combines real-time IMU streaming with posture analysis.

Works with any device orientation (phone in pocket, iPad on back, etc.)
by using gravity-vector-angle for tilt detection instead of axis-specific pitch.

Reuses complementary filter from pelvis_imu_analyzer_raw.py
and live streaming architecture from live_imu_plot.py.
"""

import asyncio
import sys
import os
import time as _time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Add arvos-sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))

from arvos import ArvosServer, IMUData


class IMUOrientationFilter:
    """Complementary filter for fusing gyro and accelerometer data into orientation.
    Adapted from pelvis_imu_analyzer_raw.py for real-time use.
    
    Uses gravity vector (from iOS sensor fusion) instead of raw accel
    for more accurate pitch/roll estimation.
    """
    
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initialized = False
    
    def update(self, gravity_x, gravity_y, gravity_z,
               gyro_x, gyro_y, gyro_z, dt):
        """
        Update orientation estimate.
        
        Uses gravity vector for roll/pitch correction (more stable than
        raw accel since iOS already filters it).
        
        Phone-in-pocket orientation:
        - Phone Y axis points up (along body)
        - Phone X axis points left/right
        - Phone Z axis points forward/back
        """
        if dt <= 0 or dt > 1.0:
            dt = 0.01  # fallback to 100Hz
        
        # Gyro integration (remapped for phone-in-pocket)
        gyro_roll_rate = gyro_z    # Roll = rotation around forward axis
        gyro_pitch_rate = gyro_x   # Pitch = rotation around side axis
        gyro_yaw_rate = gyro_y     # Yaw = rotation around vertical axis
        
        # Integrate gyroscope (predict step)
        self.roll += gyro_roll_rate * dt
        self.pitch += gyro_pitch_rate * dt
        self.yaw += gyro_yaw_rate * dt
        
        # Gravity-based angle estimation (correct step)
        # Gravity vector points "down" relative to device
        accel_roll = np.arctan2(gravity_x, -gravity_y)
        accel_pitch = np.arctan2(gravity_z, np.sqrt(gravity_x**2 + gravity_y**2))
        
        if not self.initialized:
            # First sample: initialize from gravity directly
            self.roll = accel_roll
            self.pitch = accel_pitch
            self.yaw = 0.0
            self.initialized = True
        else:
            # Complementary filter: trust gyro short-term, gravity long-term
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
            # Yaw: gyro only (no magnetometer correction)
        
        return np.degrees(self.roll), np.degrees(self.pitch), np.degrees(self.yaw)


class LiveSquatVisualizer:
    """Real-time squat depth and torso rotation visualizer."""
    
    def __init__(self, max_points=1000):
        self.max_points = max_points  # 10 seconds at 100Hz
        
        # ── Thresholds (edit these, not magic numbers below) ──────────
        self.squat_threshold_pct = 60.0    # min % to count as squatting
        self.deep_squat_pct = 60.0        # % at which squat text turns red
        self.rotation_warn_deg = 20.0     # degrees at which rotation text turns red
        self.max_tilt_deg = 45.0          # tilt from standing = 100% squat
        self.plot_window_sec = 10         # seconds of data visible on plots
        
        # ── Timing ────────────────────────────────────────────────────
        self.pre_calibration_delay = 3.0  # seconds to put on backpack
        self.calibration_duration = 3.0   # seconds of standing-still cal
        
        # ── Internal state ────────────────────────────────────────────
        # Orientation filter
        self.ori_filter = IMUOrientationFilter(alpha=0.98)
        self.last_timestamp_ns = None
        
        # Pre-calibration
        self.pre_calibrating = True
        self.pre_calibration_start = None
        
        # Calibration (gravity vector based -- device-orientation-agnostic)
        self.calibrating = False  # starts after pre-cal delay
        self.calibration_start = None
        self.calibration_gravities = []
        self.ref_gravity = None
        
        # Data buffers
        self.timestamps = deque(maxlen=max_points)
        self.rolls = deque(maxlen=max_points)
        self.pitches = deque(maxlen=max_points)
        self.yaws = deque(maxlen=max_points)
        self.squat_depths = deque(maxlen=max_points)
        self.torso_rotations = deque(maxlen=max_points)
        
        # Yaw drift correction: running baseline
        self.yaw_baseline = deque(maxlen=500)  # 5 second running mean
        
        # Stats
        self.data_count = 0
        self.start_time = None
        self.is_connected = False
        
        # Setup plot
        self._setup_plot()
    
    def _setup_plot(self):
        """Create the 4-panel visualization."""
        self.fig, axes = plt.subplots(4, 1, figsize=(14, 12))
        self.fig.suptitle('Live Squat Visualizer - Waiting for connection...',
                         fontsize=16, fontweight='bold', color='gray')
        
        # Panel 1: Orientation (Roll/Pitch/Yaw)
        self.ax_ori = axes[0]
        self.line_roll, = self.ax_ori.plot([], [], 'r-', linewidth=1.5, alpha=0.7, label='Roll')
        self.line_pitch, = self.ax_ori.plot([], [], 'g-', linewidth=2, label='Pitch')
        self.line_yaw, = self.ax_ori.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label='Yaw')
        self.ax_ori.set_ylabel('Angle (degrees)', fontsize=11)
        self.ax_ori.set_title('Orientation (Roll / Pitch / Yaw)', fontweight='bold')
        self.ax_ori.legend(loc='upper right')
        self.ax_ori.grid(True, alpha=0.3)
        self.ax_ori.set_ylim(-90, 90)
        
        # Panel 2: Squat Depth
        self.ax_squat = axes[1]
        self.line_squat, = self.ax_squat.plot([], [], 'b-', linewidth=2)
        self.squat_threshold_line = self.ax_squat.axhline(
            y=self.squat_threshold_pct, color='r', linestyle='--', linewidth=1.5, alpha=0.7,
            label=f'Squat threshold ({self.squat_threshold_pct:.0f}%)')
        self.ax_squat.set_ylabel('Squat Depth (%)', fontsize=11)
        self.ax_squat.set_title('Squat Depth (from Back Tilt)', fontweight='bold')
        self.ax_squat.legend(loc='upper right')
        self.ax_squat.grid(True, alpha=0.3)
        self.ax_squat.set_ylim(0, 60)
        
        # Panel 3: Torso Rotation
        self.ax_rotation = axes[2]
        self.line_rotation, = self.ax_rotation.plot([], [], 'g-', linewidth=2)
        self.ax_rotation.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_rotation.set_ylabel('Torso Rotation (deg)', fontsize=11)
        self.ax_rotation.set_title('Torso Rotation (drift-corrected Yaw)', fontweight='bold')
        self.ax_rotation.grid(True, alpha=0.3)
        self.ax_rotation.set_ylim(-45, 45)
        
        # Panel 4: Live Status Dashboard
        self.ax_status = axes[3]
        self.ax_status.set_xlim(0, 10)
        self.ax_status.set_ylim(0, 10)
        self.ax_status.axis('off')
        self.ax_status.set_title('Live Status', fontweight='bold')
        
        # Status text elements
        self.status_texts = {}
        labels = [
            ('squat_label', 0.5, 8, 'SQUAT DEPTH:', 14, 'left', 'gray'),
            ('squat_val', 4.5, 8, '0.0%', 28, 'left', 'blue'),
            ('rotation_label', 0.5, 5, 'TORSO ROTATION:', 14, 'left', 'gray'),
            ('rotation_val', 4.5, 5, '0.0 deg', 28, 'left', 'green'),
            ('state_label', 0.5, 2, 'STATE:', 14, 'left', 'gray'),
            ('state_val', 4.5, 2, 'STANDING', 24, 'left', 'black'),
            ('rate_label', 7, 8, 'Rate:', 11, 'left', 'gray'),
            ('rate_val', 8.5, 8, '-- Hz', 11, 'left', 'gray'),
            ('samples_label', 7, 6, 'Samples:', 11, 'left', 'gray'),
            ('samples_val', 8.5, 6, '0', 11, 'left', 'gray'),
            ('duration_label', 7, 4, 'Duration:', 11, 'left', 'gray'),
            ('duration_val', 8.5, 4, '0.0s', 11, 'left', 'gray'),
        ]
        for name, x, y, text, size, ha, color in labels:
            self.status_texts[name] = self.ax_status.text(
                x, y, text, fontsize=size, ha=ha, va='center',
                fontweight='bold', color=color,
                fontfamily='monospace' if 'val' in name else 'sans-serif'
            )
        
        # Squat depth bar (visual gauge)
        self.squat_bar = self.ax_status.barh(
            8, 0, height=1.5, left=4.5, color='blue', alpha=0.3)
        
        plt.tight_layout()
    
    def _gravity_tilt_angle(self, gx, gy, gz):
        """Compute tilt angle (degrees) between current gravity and reference gravity.
        
        Device-orientation-agnostic: works regardless of how the iPad/phone
        is mounted. Returns 0 when standing in calibrated position, increases
        as you tilt away (e.g. lean forward during squat).
        """
        if self.ref_gravity is None:
            return 0.0
        
        g_cur = np.array([gx, gy, gz])
        g_ref = self.ref_gravity
        
        # Angle = arccos(dot(a, b) / (|a| * |b|))
        dot = np.dot(g_cur, g_ref)
        mag_cur = np.linalg.norm(g_cur)
        mag_ref = np.linalg.norm(g_ref)
        
        if mag_cur < 0.01 or mag_ref < 0.01:
            return 0.0
        
        cos_angle = np.clip(dot / (mag_cur * mag_ref), -1.0, 1.0)
        return np.degrees(np.arccos(cos_angle))
    
    def add_data(self, data: IMUData):
        """Process incoming IMU sample through orientation filter and squat detection."""
        # Compute dt
        if self.last_timestamp_ns is None:
            dt = 0.01
            self.start_time = data.timestamp_ns
        else:
            dt = (data.timestamp_ns - self.last_timestamp_ns) / 1e9
        self.last_timestamp_ns = data.timestamp_ns
        
        # Get gravity vector (from iOS sensor fusion)
        if data.gravity is not None:
            gx, gy, gz = data.gravity
        else:
            # Fallback: reconstruct from accel (less accurate)
            gx, gy, gz = data.linear_acceleration
        
        # Get gyro
        wx, wy, wz = data.angular_velocity
        
        # Run complementary filter (for orientation visualization)
        roll, pitch, yaw = self.ori_filter.update(gx, gy, gz, wx, wy, wz, dt)
        
        # Timestamp relative to start
        t = (data.timestamp_ns - self.start_time) / 1e9
        
        # Phase 1: Pre-calibration delay (put on your backpack)
        if self.pre_calibrating:
            if self.pre_calibration_start is None:
                self.pre_calibration_start = t
            
            elapsed = t - self.pre_calibration_start
            if elapsed >= self.pre_calibration_delay:
                # Done waiting, move to calibration
                self.pre_calibrating = False
                self.calibrating = True
                print("Pre-calibration delay done. CALIBRATING NOW -- stand still!")
            
            # Store data but no squat/rotation yet
            squat_depth = 0.0
            torso_rotation = 0.0
        
        # Phase 2: Calibration (collect standing gravity reference)
        elif self.calibrating:
            if self.calibration_start is None:
                self.calibration_start = t
            
            self.calibration_gravities.append([gx, gy, gz])
            
            if t - self.calibration_start >= self.calibration_duration:
                # Compute reference gravity vector (median for robustness)
                gravs = np.array(self.calibration_gravities)
                self.ref_gravity = np.median(gravs, axis=0)
                self.calibrating = False
                tilt_check = np.linalg.norm(self.ref_gravity)
                print(f"Calibration complete! Reference gravity magnitude: {tilt_check:.3f} g")
                print(f"Reference vector: [{self.ref_gravity[0]:.3f}, {self.ref_gravity[1]:.3f}, {self.ref_gravity[2]:.3f}]")
            
            squat_depth = 0.0
            torso_rotation = 0.0
        
        # Phase 3: Live tracking
        else:
            # Squat depth from gravity tilt angle (device-orientation-agnostic)
            tilt_deg = self._gravity_tilt_angle(gx, gy, gz)
            squat_depth = (tilt_deg / self.max_tilt_deg) * 100
            squat_depth = np.clip(squat_depth, 0, 100)
            
            # Torso rotation: drift-corrected yaw
            self.yaw_baseline.append(yaw)
            yaw_mean = np.mean(self.yaw_baseline)
            torso_rotation = yaw - yaw_mean
        
        # Store data
        self.timestamps.append(t)
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.yaws.append(yaw)
        self.squat_depths.append(squat_depth)
        self.torso_rotations.append(torso_rotation)
        
        self.data_count += 1
    
    def update_plot(self, frame):
        """Update all panels with latest data."""
        if len(self.timestamps) < 2:
            return ()
        
        t = np.array(self.timestamps)
        x_max = t[-1]
        x_min = max(0, x_max - self.plot_window_sec)
        
        # Panel 1: Orientation
        self.line_roll.set_data(t, np.array(self.rolls))
        self.line_pitch.set_data(t, np.array(self.pitches))
        self.line_yaw.set_data(t, np.array(self.yaws))
        self.ax_ori.set_xlim(x_min, x_max)
        
        # Auto-scale Y for orientation
        if len(self.pitches) > 10:
            all_angles = list(self.rolls) + list(self.pitches) + list(self.yaws)
            y_min = min(all_angles) - 5
            y_max = max(all_angles) + 5
            self.ax_ori.set_ylim(max(-180, y_min), min(180, y_max))
        
        # Panel 2: Squat Depth
        squat_arr = np.array(self.squat_depths)
        self.line_squat.set_data(t, squat_arr)
        self.ax_squat.set_xlim(x_min, x_max)
        
        # Auto-scale squat Y
        if len(squat_arr) > 10:
            sq_max = max(np.max(squat_arr) + 5, 20)
            self.ax_squat.set_ylim(0, min(sq_max, 100))
        
        # Panel 3: Torso Rotation
        rot_arr = np.array(self.torso_rotations)
        self.line_rotation.set_data(t, rot_arr)
        self.ax_rotation.set_xlim(x_min, x_max)
        
        # Auto-scale rotation Y
        if len(rot_arr) > 10:
            rot_abs_max = max(np.max(np.abs(rot_arr)) + 5, 10)
            self.ax_rotation.set_ylim(-rot_abs_max, rot_abs_max)
        
        # Panel 4: Live Status
        current_squat = squat_arr[-1] if len(squat_arr) > 0 else 0
        current_rotation = rot_arr[-1] if len(rot_arr) > 0 else 0
        is_squatting = current_squat > self.squat_threshold_pct
        
        # Update text
        self.status_texts['squat_val'].set_text(f'{current_squat:.1f}%')
        self.status_texts['squat_val'].set_color('red' if current_squat > self.deep_squat_pct else 'blue')
        
        self.status_texts['rotation_val'].set_text(f'{current_rotation:.1f} deg')
        self.status_texts['rotation_val'].set_color(
            'red' if abs(current_rotation) > self.rotation_warn_deg else 'green')
        
        state = 'SQUATTING' if is_squatting else 'STANDING'
        self.status_texts['state_val'].set_text(state)
        self.status_texts['state_val'].set_color('red' if is_squatting else 'green')
        
        # Data rate
        dt_total = t[-1] - t[0]
        rate = len(t) / dt_total if dt_total > 0 else 0
        self.status_texts['rate_val'].set_text(f'{rate:.0f} Hz')
        self.status_texts['samples_val'].set_text(f'{self.data_count}')
        self.status_texts['duration_val'].set_text(f'{t[-1]:.1f}s')
        
        # Update title based on phase
        if self.pre_calibrating:
            elapsed = t[-1] - (self.pre_calibration_start or 0)
            remaining = max(0, self.pre_calibration_delay - elapsed)
            self.fig.suptitle(
                f'Live Squat Visualizer - GET READY ({remaining:.1f}s) - put on backpack!',
                fontsize=16, fontweight='bold', color='blue')
        elif self.calibrating:
            elapsed = t[-1] - (self.calibration_start or 0)
            remaining = max(0, self.calibration_duration - elapsed)
            self.fig.suptitle(
                f'Live Squat Visualizer - CALIBRATING (stand still... {remaining:.1f}s)',
                fontsize=16, fontweight='bold', color='orange')
        else:
            self.fig.suptitle(
                f'Live Squat Visualizer | {state} | Squat: {current_squat:.0f}% | Rotation: {current_rotation:.0f} deg',
                fontsize=16, fontweight='bold',
                color='red' if is_squatting else 'green')
        
        return ()


# Global visualizer
viz = LiveSquatVisualizer()


def run_server():
    """Run asyncio server in a separate thread."""
    async def server_main():
        server = ArvosServer(host="0.0.0.0", port=8765)
        server.print_qr_code()
        
        def on_imu(data: IMUData):
            viz.add_data(data)
        
        async def on_connect(client_id: str):
            viz.is_connected = True
            print(f"Connected: {client_id}")
            print("3 seconds to put on backpack, then 3 seconds of calibration (stand still)...")
        
        async def on_disconnect(client_id: str):
            viz.is_connected = False
            print(f"Disconnected: {client_id}")
        
        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        
        print("Starting squat visualizer server on port 8765...")
        print("Connect your iPad, then: 3s to put on backpack -> 3s calibration (stand still)\n")
        
        await server.start()
    
    asyncio.run(server_main())


if __name__ == "__main__":
    try:
        # Start server in background thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        _time.sleep(1)
        
        # Start animation in main thread
        ani = FuncAnimation(viz.fig, viz.update_plot, interval=50,
                           blit=False, cache_frame_data=False)
        plt.show()
        
    except KeyboardInterrupt:
        print("\n\nStopped")
