#!/usr/bin/env python3
"""
Live Dual-IMU Visualizer
Streams from two phones simultaneously:
  - Board IMU (port 9090): lean, slope, vibration (from live_board_visualizer)
  - Body IMU (port 9091): squat depth, torso rotation, orientation (from live_squat_visualizer)

Processing matches the single-IMU visualizers exactly.
Calibration is basic for now (Checkpoints 2-3 add full calibration).
"""

import asyncio
import sys
import os
import time as _time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))
from arvos import ArvosServer, IMUData


# ── Complementary Filter (from live_squat_visualizer.py) ────────────────────

class IMUOrientationFilter:
    """Complementary filter: 98% gyro + 2% gravity for orientation."""

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initialized = False

    def update(self, gx, gy, gz, wx, wy, wz, dt):
        if dt <= 0 or dt > 1.0:
            dt = 0.01

        # Gyro integration (phone-in-pocket mapping)
        self.roll += wz * dt
        self.pitch += wx * dt
        self.yaw += wy * dt

        # Gravity-based correction
        accel_roll = np.arctan2(gx, -gy)
        accel_pitch = np.arctan2(gz, np.sqrt(gx**2 + gy**2))

        if not self.initialized:
            self.roll = accel_roll
            self.pitch = accel_pitch
            self.yaw = 0.0
            self.initialized = True
        else:
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

        return np.degrees(self.roll), np.degrees(self.pitch), np.degrees(self.yaw)


# ── Dual Visualizer ─────────────────────────────────────────────────────────

class DualVisualizer:
    def __init__(self, max_points=1000):
        self.max_points = max_points
        self.plot_window_sec = 10

        # ── Board state ──────────────────────────────────────────────────
        self.board_t = deque(maxlen=max_points)
        self.board_lean = deque(maxlen=max_points)
        self.board_slope = deque(maxlen=max_points)
        self.board_vib = deque(maxlen=max_points)
        self.board_vib_raw = deque(maxlen=50)  # 0.5s window for RMS
        self.board_last_ts_ns = None
        self.board_start_ns = None
        self.board_connected = False
        self.is_goofy = False

        # Board calibration (Rodrigues rotation from live_board_visualizer)
        self.board_ref_gravity = None
        self.board_cal_gravities = []
        self.board_cal_start = None
        self.board_waiting_for_start = True
        self.board_pre_cal = False
        self.board_calibrating = False
        self.board_calibrated = False
        self.board_cal_rotation = np.eye(3)

        # ── Body state ───────────────────────────────────────────────────
        self.body_t = deque(maxlen=max_points)
        self.body_squat = deque(maxlen=max_points)
        self.body_torso_rot = deque(maxlen=max_points)
        self.body_roll = deque(maxlen=max_points)
        self.body_pitch = deque(maxlen=max_points)
        self.body_yaw = deque(maxlen=max_points)
        self.body_last_ts_ns = None
        self.body_start_ns = None
        self.body_connected = False

        # Complementary filter (from squat visualizer)
        self.ori_filter = IMUOrientationFilter(alpha=0.98)

        # Squat: gravity-vector tilt angle (device-agnostic)
        self.body_ref_gravity = None
        self.body_cal_gravities = []
        self.body_cal_start = None
        self.body_waiting_for_start = True  # Wait for 's' key
        self.body_pre_cal = False
        self.body_calibrating = False
        self.body_calibrated = False
        self.pre_cal_delay = 3.0
        self.cal_duration = 3.0
        self.max_tilt_deg = 45.0  # tilt at which squat = 100%

        # Yaw drift correction (running baseline)
        self.yaw_baseline = deque(maxlen=500)

        self._setup_plot()

    # ── Keyboard handler ────────────────────────────────────────────────

    def _on_key(self, event):
        if event.key == 'k':
            print("\n⚡ Kill requested -- closing visualizer")
            plt.close('all')
        elif event.key == 'g':
            self.is_goofy = not self.is_goofy
            stance = 'GOOFY' if self.is_goofy else 'REGULAR'
            print(f"\n🛹 Stance toggled to {stance}")
        elif event.key == 'c':
            self._start_calibration()

    def _start_calibration(self):
        # Reset board calibration
        self.board_ref_gravity = None
        self.board_cal_gravities = []
        self.board_cal_start = None
        self.board_waiting_for_start = False
        self.board_pre_cal = True
        self.board_calibrating = False
        self.board_calibrated = False
        self.board_cal_rotation = np.eye(3)

        # Reset body calibration
        self.body_ref_gravity = None
        self.body_cal_gravities = []
        self.body_cal_start = None
        self.body_waiting_for_start = False
        self.body_pre_cal = True
        self.body_calibrating = False
        self.body_calibrated = False
        self.yaw_baseline.clear()
        self.ori_filter = IMUOrientationFilter(alpha=0.98)

        print("\n🎯 Calibration started! Board flat + stand still. 3s delay + 3s cal...")

    # ── Rodrigues rotation (from live_board_visualizer.py) ─────────────

    def _compute_board_cal_rotation(self):
        """Rodrigues' rotation: maps ref_gravity to (0, 0, -|g|)."""
        g_ref = self.board_ref_gravity
        g_mag = np.linalg.norm(g_ref)
        if g_mag < 0.01:
            self.board_cal_rotation = np.eye(3)
            return

        g_unit = g_ref / g_mag
        target = np.array([0.0, 0.0, -1.0])

        axis = np.cross(g_unit, target)
        sin_a = np.linalg.norm(axis)
        cos_a = np.dot(g_unit, target)

        if sin_a < 1e-6:
            self.board_cal_rotation = np.eye(3) if cos_a > 0 else np.diag([-1, -1, 1])
            return

        axis = axis / sin_a
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        self.board_cal_rotation = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)

    def _decompose_gravity(self, gx, gy, gz):
        """Extract lean/slope in degrees from calibrated gravity."""
        g_cal = self.board_cal_rotation @ np.array([gx, gy, gz])
        lean_deg = np.degrees(np.arctan2(g_cal[1], -g_cal[2]))
        slope_deg = np.degrees(np.arctan2(g_cal[0], -g_cal[2]))
        if self.is_goofy:
            slope_deg = -slope_deg
        return lean_deg, slope_deg

    # ── Board data processing (matches live_board_visualizer) ────────────

    def add_board_data(self, data: IMUData):
        if self.board_last_ts_ns is None:
            self.board_start_ns = data.timestamp_ns
        self.board_last_ts_ns = data.timestamp_ns
        t = (data.timestamp_ns - self.board_start_ns) / 1e9

        # Gravity vector
        if data.gravity is not None:
            gx, gy, gz = data.gravity
        else:
            gx, gy, gz = data.linear_acceleration

        # Vibration RMS (matching board visualizer exactly)
        accel_mag = np.linalg.norm(data.linear_acceleration)
        self.board_vib_raw.append(accel_mag)
        vib_rms = np.sqrt(np.mean(np.array(self.board_vib_raw) ** 2))

        # ── Board calibration state machine ──────────────────────────
        lean_deg = 0.0
        slope_deg = 0.0

        if self.board_waiting_for_start:
            # Raw uncalibrated angles while waiting
            lean_deg = np.degrees(np.arctan2(gy, -gz))
            slope_deg = np.degrees(np.arctan2(gx, -gz))

        elif self.board_pre_cal:
            if self.board_cal_start is None:
                self.board_cal_start = t
            if t - self.board_cal_start >= self.pre_cal_delay:
                self.board_pre_cal = False
                self.board_calibrating = True
                self.board_cal_start = t
                print("📟 Board: CALIBRATING -- keep flat and still!")
            lean_deg = np.degrees(np.arctan2(gy, -gz))
            slope_deg = np.degrees(np.arctan2(gx, -gz))

        elif self.board_calibrating:
            self.board_cal_gravities.append([gx, gy, gz])
            if t - self.board_cal_start >= self.cal_duration:
                gravs = np.array(self.board_cal_gravities)
                self.board_ref_gravity = np.median(gravs, axis=0)
                self._compute_board_cal_rotation()
                self.board_calibrating = False
                self.board_calibrated = True
                g_mag = np.linalg.norm(self.board_ref_gravity)
                print(f"📟 Board calibration complete! |g| = {g_mag:.3f} m/s²")
                print(f"   Ref: [{self.board_ref_gravity[0]:.3f}, "
                      f"{self.board_ref_gravity[1]:.3f}, "
                      f"{self.board_ref_gravity[2]:.3f}]")
            lean_deg = np.degrees(np.arctan2(gy, -gz))
            slope_deg = np.degrees(np.arctan2(gx, -gz))

        elif self.board_calibrated:
            lean_deg, slope_deg = self._decompose_gravity(gx, gy, gz)

        self.board_t.append(t)
        self.board_lean.append(lean_deg)
        self.board_slope.append(slope_deg)
        self.board_vib.append(vib_rms)

    # ── Body data processing (matches live_squat_visualizer) ─────────────

    def add_body_data(self, data: IMUData):
        if self.body_last_ts_ns is None:
            self.body_start_ns = data.timestamp_ns
            dt = 0.01
        else:
            dt = (data.timestamp_ns - self.body_last_ts_ns) / 1e9
        self.body_last_ts_ns = data.timestamp_ns
        t = (data.timestamp_ns - self.body_start_ns) / 1e9

        # Gravity vector
        if data.gravity is not None:
            gx, gy, gz = data.gravity
        else:
            gx, gy, gz = data.linear_acceleration

        # Gyro
        wx, wy, wz = data.angular_velocity

        # Complementary filter for orientation
        roll, pitch, yaw = self.ori_filter.update(gx, gy, gz, wx, wy, wz, dt)

        # ── Calibration state machine ────────────────────────────────
        squat_depth = 0.0
        torso_rotation = 0.0

        if self.body_waiting_for_start:
            pass  # Wait for 's' key

        elif self.body_pre_cal:
            if self.body_cal_start is None:
                self.body_cal_start = t
            if t - self.body_cal_start >= self.pre_cal_delay:
                self.body_pre_cal = False
                self.body_calibrating = True
                self.body_cal_start = t
                print("Body: CALIBRATING -- stand still!")

        elif self.body_calibrating:
            self.body_cal_gravities.append([gx, gy, gz])
            if t - self.body_cal_start >= self.cal_duration:
                gravs = np.array(self.body_cal_gravities)
                self.body_ref_gravity = np.median(gravs, axis=0)
                self.body_calibrating = False
                self.body_calibrated = True
                mag = np.linalg.norm(self.body_ref_gravity)
                print(f"Body calibration complete! |g| = {mag:.3f}")

        elif self.body_calibrated:
            # Squat depth from gravity-vector tilt angle (device-agnostic)
            g_cur = np.array([gx, gy, gz])
            dot = np.dot(g_cur, self.body_ref_gravity)
            mag_cur = np.linalg.norm(g_cur)
            mag_ref = np.linalg.norm(self.body_ref_gravity)
            if mag_cur > 0.01 and mag_ref > 0.01:
                cos_angle = np.clip(dot / (mag_cur * mag_ref), -1.0, 1.0)
                tilt_deg = np.degrees(np.arccos(cos_angle))
                squat_depth = np.clip((tilt_deg / self.max_tilt_deg) * 100, 0, 100)

            # Torso rotation: drift-corrected yaw
            self.yaw_baseline.append(yaw)
            yaw_mean = np.mean(self.yaw_baseline)
            torso_rotation = yaw - yaw_mean

        self.body_t.append(t)
        self.body_squat.append(squat_depth)
        self.body_torso_rot.append(torso_rotation)
        self.body_roll.append(roll)
        self.body_pitch.append(pitch)
        self.body_yaw.append(yaw)

    # ── Plot setup ───────────────────────────────────────────────────────

    def _setup_plot(self):
        self.fig, axes = plt.subplots(3, 2, figsize=(15, 10))
        self.fig.suptitle('Dual-IMU -- Waiting for connections...', fontsize=14, fontweight='bold')

        # Keyboard handler
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # Top-left: Board Lean
        ax = axes[0, 0]
        self.line_lean, = ax.plot([], [], 'b-', linewidth=1.5)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax.set_title('Board Lean (deg)', fontweight='bold')
        ax.set_ylabel('Angle (deg)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-30, 30)
        self.ax_lean = ax

        # Top-right: Board Slope
        ax = axes[0, 1]
        self.line_slope, = ax.plot([], [], 'g-', linewidth=1.5)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax.set_title('Board Slope (deg)', fontweight='bold')
        ax.set_ylabel('Angle (deg)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-20, 20)
        self.ax_slope = ax

        # Mid-left: Board Vibration
        ax = axes[1, 0]
        self.line_vib, = ax.plot([], [], 'm-', linewidth=1.5)
        ax.set_title('Board Vibration', fontweight='bold')
        ax.set_ylabel('RMS (m/s²)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 5)
        self.ax_vib = ax

        # Mid-right: Squat Depth
        ax = axes[1, 1]
        self.line_squat, = ax.plot([], [], 'b-', linewidth=2)
        ax.axhline(y=60, color='r', linestyle='--', linewidth=1, alpha=0.5, label='Threshold')
        ax.set_title('Squat Depth (%)', fontweight='bold')
        ax.set_ylabel('Depth (%)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 100)
        self.ax_squat = ax

        # Bottom-left: Torso Rotation
        ax = axes[2, 0]
        self.line_torso, = ax.plot([], [], 'g-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax.set_title('Torso Rotation (drift-corrected)', fontweight='bold')
        ax.set_ylabel('Rotation (deg)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-45, 45)
        self.ax_torso = ax

        # Bottom-right: Body Orientation (roll/pitch/yaw)
        ax = axes[2, 1]
        self.line_broll, = ax.plot([], [], 'r-', linewidth=1, alpha=0.7, label='Roll')
        self.line_bpitch, = ax.plot([], [], 'g-', linewidth=1.5, label='Pitch')
        self.line_byaw, = ax.plot([], [], 'b-', linewidth=1, alpha=0.7, label='Yaw')
        ax.set_title('Body Orientation', fontweight='bold')
        ax.set_ylabel('Angle (deg)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-90, 90)
        self.ax_ori = ax

        plt.tight_layout()

    # ── Animation update (uses set_data, not clear+plot) ─────────────────

    def update_plot(self, frame):
        # Board plots
        if len(self.board_t) > 2:
            t = np.array(self.board_t)
            x_max = t[-1]
            x_min = max(0, x_max - self.plot_window_sec)

            self.line_lean.set_data(t, np.array(self.board_lean))
            self.ax_lean.set_xlim(x_min, x_max)
            lean_val = self.board_lean[-1]
            self.ax_lean.set_title(f'Board Lean: {lean_val:+.1f} deg', fontweight='bold')

            self.line_slope.set_data(t, np.array(self.board_slope))
            self.ax_slope.set_xlim(x_min, x_max)
            slope_val = self.board_slope[-1]
            self.ax_slope.set_title(f'Board Slope: {slope_val:+.1f} deg', fontweight='bold')

            self.line_vib.set_data(t, np.array(self.board_vib))
            self.ax_vib.set_xlim(x_min, x_max)
            vib_val = self.board_vib[-1]
            self.ax_vib.set_title(f'Vibration: {vib_val:.2f} m/s²', fontweight='bold')

        # Body plots
        if len(self.body_t) > 2:
            t = np.array(self.body_t)
            x_max = t[-1]
            x_min = max(0, x_max - self.plot_window_sec)

            self.line_squat.set_data(t, np.array(self.body_squat))
            self.ax_squat.set_xlim(x_min, x_max)
            sq = self.body_squat[-1]
            self.ax_squat.set_title(f'Squat Depth: {sq:.0f}%', fontweight='bold')

            self.line_torso.set_data(t, np.array(self.body_torso_rot))
            self.ax_torso.set_xlim(x_min, x_max)
            rot = self.body_torso_rot[-1]
            self.ax_torso.set_title(f'Torso Rotation: {rot:+.1f} deg', fontweight='bold')

            self.line_broll.set_data(t, np.array(self.body_roll))
            self.line_bpitch.set_data(t, np.array(self.body_pitch))
            self.line_byaw.set_data(t, np.array(self.body_yaw))
            self.ax_ori.set_xlim(x_min, x_max)

        # Title
        b_conn = "✓" if self.board_connected else "✗"
        d_conn = "✓" if self.body_connected else "✗"

        # Board cal status
        if self.board_waiting_for_start:
            b_cal = "uncal"
        elif self.board_pre_cal:
            b_cal = "wait..."
        elif self.board_calibrating:
            b_cal = "CAL!"
        elif self.board_calibrated:
            b_cal = "cal✓"
        else:
            b_cal = ""

        # Body cal status
        if self.body_waiting_for_start:
            d_cal = "uncal"
        elif self.body_pre_cal:
            d_cal = "wait..."
        elif self.body_calibrating:
            d_cal = "CAL!"
        elif self.body_calibrated:
            d_cal = "cal✓"
        else:
            d_cal = ""

        stance = "G" if self.is_goofy else "R"
        prompt = " | Press 'c' to calibrate" if (self.board_waiting_for_start or self.body_waiting_for_start) else ""
        self.fig.suptitle(
            f"BOARD {b_conn}({b_cal}) | BODY {d_conn}({d_cal}) | {stance}{prompt} | 'k'=quit 'g'=stance",
            fontsize=12, fontweight='bold')

        return ()


# ── Global instance ──────────────────────────────────────────────────────────

viz = DualVisualizer()


# ── Server threads (each with own asyncio.run, matching single-IMU pattern) ──

def run_board_server():
    async def server_main():
        server = ArvosServer("0.0.0.0", 9090)

        def on_imu(data: IMUData):
            viz.add_board_data(data)

        async def on_connect(client_id):
            viz.board_connected = True
            print(f"📟 Board connected: {client_id}")

        async def on_disconnect(client_id):
            viz.board_connected = False
            print(f"📟 Board disconnected: {client_id}")

        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        print("Board server listening on port 9090...")
        await server.start()

    asyncio.run(server_main())


def run_body_server():
    async def server_main():
        server = ArvosServer("0.0.0.0", 9091)

        def on_imu(data: IMUData):
            viz.add_body_data(data)

        async def on_connect(client_id):
            viz.body_connected = True
            print(f"🧍 Body connected: {client_id}")

        async def on_disconnect(client_id):
            viz.body_connected = False
            print(f"🧍 Body disconnected: {client_id}")

        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        print("Body server listening on port 9091...")
        await server.start()

    asyncio.run(server_main())


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("="*60)
    print("DUAL-IMU REAL-TIME VISUALIZER")
    print("="*60)
    print("Board IMU: ws://<laptop-ip>:9090")
    print("Body IMU:  ws://<laptop-ip>:9091")
    print("="*60)

    try:
        board_thread = threading.Thread(target=run_board_server, daemon=True)
        body_thread = threading.Thread(target=run_body_server, daemon=True)
        board_thread.start()
        body_thread.start()

        _time.sleep(1)

        ani = FuncAnimation(viz.fig, viz.update_plot, interval=50,
                           blit=False, cache_frame_data=False)
        plt.show()

    except KeyboardInterrupt:
        print("\n\nStopped")
