#!/usr/bin/env python3
"""
Live Board IMU Visualizer
Streams IMU data from a phone/iPad duct-taped to a skateboard deck and
visualizes lean angle (heelside/toeside), slope, and vibration in real time.

Device mounting: screen up, top of device toward toes.
  - Device Y axis -> across the board toward toeside rail
  - Device X axis -> along the board (nose/tail)
  - Device Z axis -> up from the deck

Calibration: place board flat on ground for 3 seconds to zero out mounting tilt.
"""

import asyncio
import enum
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


# ── Calibration state machine ────────────────────────────────────────────────

class CalPhase(enum.Enum):
    PRE_CAL = "pre_cal"          # delay before calibration (place board flat)
    FLAT_CAL = "flat_cal"        # collecting flat-ground gravity reference
    # Future: HEEL_CAL, TOE_CAL for range calibration
    LIVE = "live"                # tracking


# ── Board Visualizer ─────────────────────────────────────────────────────────

class BoardVisualizer:
    """Real-time skateboard lean, slope, and vibration visualizer."""

    def __init__(self, max_points=1000):
        self.max_points = max_points  # 10 seconds at 100Hz

        # ── Thresholds (edit these, not magic numbers below) ──────────
        self.lean_flat_threshold_deg = 2.0   # below this = "flat"
        self.lean_warn_deg = 15.0            # edge depth where lean text turns red
        self.slope_warn_deg = 10.0           # slope where text turns orange
        self.vibration_window = 50           # samples for RMS (~0.5s at 100Hz)
        self.plot_window_sec = 10            # visible time window on plots

        # ── Timing ────────────────────────────────────────────────────
        self.pre_cal_delay = 3.0             # seconds to place board flat
        self.cal_duration = 3.0              # seconds of flat-ground collection

        # ── User settings ─────────────────────────────────────────────
        self.is_goofy = False                # toggled with 'g' key

        # ── Calibration state ─────────────────────────────────────────
        self.cal_phase = CalPhase.PRE_CAL
        self.phase_start_t = None            # timestamp when current phase began
        self.cal_gravities = []              # gravity vectors during flat cal
        self.ref_gravity = None              # median gravity on flat ground
        self.cal_rotation = np.eye(3)        # rotation matrix: device -> board frame

        # ── Data buffers ──────────────────────────────────────────────
        self.timestamps = deque(maxlen=max_points)
        self.leans = deque(maxlen=max_points)
        self.slopes = deque(maxlen=max_points)
        self.vibrations = deque(maxlen=max_points)
        self.vibration_raw = deque(maxlen=self.vibration_window)  # raw accel mags for RMS

        # ── Stats ─────────────────────────────────────────────────────
        self.data_count = 0
        self.start_time = None
        self.last_timestamp_ns = None
        self.is_connected = False

        # ── Plot ──────────────────────────────────────────────────────
        self._setup_plot()

    # ── Calibration helpers ───────────────────────────────────────────────

    def _compute_cal_rotation(self):
        """Compute rotation matrix that maps ref_gravity to (0, 0, -|g|).

        After applying this rotation, the calibrated gravity on flat ground
        becomes purely in the -Z direction, so lean (Y component) and slope
        (X component) are both zero on flat ground.
        Uses Rodrigues' rotation formula.
        """
        g_ref = self.ref_gravity
        g_mag = np.linalg.norm(g_ref)
        if g_mag < 0.01:
            self.cal_rotation = np.eye(3)
            return

        g_unit = g_ref / g_mag
        target = np.array([0.0, 0.0, -1.0])  # "down" in board frame

        axis = np.cross(g_unit, target)
        sin_a = np.linalg.norm(axis)
        cos_a = np.dot(g_unit, target)

        if sin_a < 1e-6:
            # Already aligned (or 180-deg flip)
            self.cal_rotation = np.eye(3) if cos_a > 0 else np.diag([-1.0, -1.0, 1.0])
            return

        axis = axis / sin_a  # normalize rotation axis

        # Skew-symmetric matrix of axis
        K = np.array([
            [0,       -axis[2],  axis[1]],
            [axis[2],  0,       -axis[0]],
            [-axis[1], axis[0],  0      ],
        ])
        self.cal_rotation = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)

    def _decompose_gravity(self, gx, gy, gz):
        """Decompose gravity into lean angle and slope angle using calibrated rotation.

        Returns (lean_deg, slope_deg):
          lean_deg:  positive = toeside, negative = heelside
          slope_deg: positive = nose up / uphill, negative = nose down / downhill
        """
        g_dev = np.array([gx, gy, gz])
        g_cal = self.cal_rotation @ g_dev  # rotate to board frame

        # In board frame after cal rotation:
        #   g_cal[0] = along board (X), g_cal[1] = toward toes (Y), g_cal[2] = down (Z)
        lean_deg = np.degrees(np.arctan2(g_cal[1], -g_cal[2]))

        slope_deg = np.degrees(np.arctan2(g_cal[0], -g_cal[2]))
        if self.is_goofy:
            slope_deg = -slope_deg  # flip nose/tail for goofy

        return lean_deg, slope_deg

    # ── Data processing ──────────────────────────────────────────────────

    def add_data(self, data: IMUData):
        """Process one incoming IMU sample."""
        # Compute dt
        if self.last_timestamp_ns is None:
            self.start_time = data.timestamp_ns
            dt = 0.01
        else:
            dt = (data.timestamp_ns - self.last_timestamp_ns) / 1e9
        self.last_timestamp_ns = data.timestamp_ns

        # Timestamp relative to start
        t = (data.timestamp_ns - self.start_time) / 1e9

        # Gravity vector (prefer iOS sensor-fused gravity)
        if data.gravity is not None:
            gx, gy, gz = data.gravity
        else:
            gx, gy, gz = data.linear_acceleration

        # Vibration: always compute, even during calibration
        accel_mag = np.linalg.norm(data.linear_acceleration)
        self.vibration_raw.append(accel_mag)
        vib_rms = np.sqrt(np.mean(np.array(self.vibration_raw) ** 2))

        # ── State machine ─────────────────────────────────────────────
        lean_deg = 0.0
        slope_deg = 0.0

        if self.cal_phase == CalPhase.PRE_CAL:
            if self.phase_start_t is None:
                self.phase_start_t = t
            elapsed = t - self.phase_start_t
            if elapsed >= self.pre_cal_delay:
                self.cal_phase = CalPhase.FLAT_CAL
                self.phase_start_t = t
                print("Pre-cal delay done. CALIBRATING -- keep board flat and still!")

        elif self.cal_phase == CalPhase.FLAT_CAL:
            self.cal_gravities.append([gx, gy, gz])
            elapsed = t - self.phase_start_t
            if elapsed >= self.cal_duration:
                gravs = np.array(self.cal_gravities)
                self.ref_gravity = np.median(gravs, axis=0)
                self._compute_cal_rotation()
                self.cal_phase = CalPhase.LIVE
                g_mag = np.linalg.norm(self.ref_gravity)
                print(f"Calibration complete! |g| = {g_mag:.3f} m/s^2")
                print(f"Ref gravity: [{self.ref_gravity[0]:.3f}, {self.ref_gravity[1]:.3f}, {self.ref_gravity[2]:.3f}]")

        elif self.cal_phase == CalPhase.LIVE:
            lean_deg, slope_deg = self._decompose_gravity(gx, gy, gz)

        # Store data
        self.timestamps.append(t)
        self.leans.append(lean_deg)
        self.slopes.append(slope_deg)
        self.vibrations.append(vib_rms)
        self.data_count += 1

    # ── Plot setup ───────────────────────────────────────────────────────

    def _setup_plot(self):
        """Create the 4-panel visualization."""
        self.fig, axes = plt.subplots(4, 1, figsize=(14, 12))
        self.fig.suptitle('Board Visualizer -- Waiting for connection...',
                         fontsize=16, fontweight='bold', color='gray')

        # Connect keyboard handler for goofy toggle
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # Panel 1: Lean Angle ──────────────────────────────────────────
        self.ax_lean = axes[0]
        self.line_lean, = self.ax_lean.plot([], [], 'b-', linewidth=2)
        self.ax_lean.axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.4)
        # Flat zone shading
        self.ax_lean.axhspan(
            -self.lean_flat_threshold_deg, self.lean_flat_threshold_deg,
            color='gray', alpha=0.1, label=f'Flat zone (+/-{self.lean_flat_threshold_deg:.0f} deg)')
        self.ax_lean.set_ylabel('Lean Angle (deg)', fontsize=11)
        self.ax_lean.set_title('Lean Angle (+ toeside / - heelside)', fontweight='bold')
        self.ax_lean.legend(loc='upper right')
        self.ax_lean.grid(True, alpha=0.3)
        self.ax_lean.set_ylim(-30, 30)

        # Panel 2: Slope ──────────────────────────────────────────────
        self.ax_slope = axes[1]
        self.line_slope, = self.ax_slope.plot([], [], 'g-', linewidth=2)
        self.ax_slope.axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.4)
        self.ax_slope.set_ylabel('Slope (deg)', fontsize=11)
        self.ax_slope.set_title('Slope (+ uphill / - downhill)', fontweight='bold')
        self.ax_slope.grid(True, alpha=0.3)
        self.ax_slope.set_ylim(-20, 20)

        # Panel 3: Vibration ──────────────────────────────────────────
        self.ax_vib = axes[2]
        self.line_vib, = self.ax_vib.plot([], [], 'm-', linewidth=1.5)
        self.ax_vib.set_ylabel('Vibration (m/s^2 RMS)', fontsize=11)
        self.ax_vib.set_title('Board Vibration / Road Roughness', fontweight='bold')
        self.ax_vib.grid(True, alpha=0.3)
        self.ax_vib.set_ylim(0, 5)

        # Panel 4: Status Dashboard ───────────────────────────────────
        self.ax_status = axes[3]
        self.ax_status.set_xlim(0, 10)
        self.ax_status.set_ylim(0, 10)
        self.ax_status.axis('off')
        self.ax_status.set_title('Live Status', fontweight='bold')

        self.status_texts = {}
        labels = [
            # (key,        x,   y,  default_text,    size, ha,     color)
            ('edge_label', 0.3, 8,  'EDGE:',         14,   'left', 'gray'),
            ('edge_val',   2.5, 8,  'FLAT',           28,   'left', 'black'),
            ('lean_label', 0.3, 5.5,'LEAN:',          14,   'left', 'gray'),
            ('lean_val',   2.5, 5.5,'0.0 deg',        24,   'left', 'blue'),
            ('slope_label',0.3, 3,  'SLOPE:',         14,   'left', 'gray'),
            ('slope_val',  2.5, 3,  '0.0 deg',        24,   'left', 'green'),
            ('vib_label',  0.3, 0.8,'VIBRATION:',     14,   'left', 'gray'),
            ('vib_val',    3.5, 0.8,'0.0',            18,   'left', 'purple'),
            ('stance_label', 6.5, 8,  'Stance:',      11,   'left', 'gray'),
            ('stance_val',   8.0, 8,  'REGULAR',      13,   'left', 'black'),
            ('rate_label',   6.5, 6,  'Rate:',        11,   'left', 'gray'),
            ('rate_val',     8.0, 6,  '-- Hz',        11,   'left', 'gray'),
            ('samples_label',6.5, 4,  'Samples:',     11,   'left', 'gray'),
            ('samples_val',  8.0, 4,  '0',            11,   'left', 'gray'),
            ('duration_label',6.5,2,  'Duration:',    11,   'left', 'gray'),
            ('duration_val', 8.0, 2,  '0.0s',         11,   'left', 'gray'),
            ('hint',         6.5, 0.5,"Press 'g' to toggle regular/goofy", 9, 'left', 'gray'),
        ]
        for name, x, y, text, size, ha, color in labels:
            self.status_texts[name] = self.ax_status.text(
                x, y, text, fontsize=size, ha=ha, va='center',
                fontweight='bold', color=color,
                fontfamily='monospace' if 'val' in name else 'sans-serif'
            )

        plt.tight_layout()

    # ── Keyboard handler ─────────────────────────────────────────────────

    def _on_key(self, event):
        if event.key == 'g':
            self.is_goofy = not self.is_goofy
            stance = 'GOOFY' if self.is_goofy else 'REGULAR'
            print(f"Stance toggled to {stance}")

    # ── Animation update ─────────────────────────────────────────────────

    def update_plot(self, frame):
        """Update all 4 panels with latest data."""
        if len(self.timestamps) < 2:
            return ()

        t = np.array(self.timestamps)
        x_max = t[-1]
        x_min = max(0, x_max - self.plot_window_sec)

        # Panel 1: Lean angle
        lean_arr = np.array(self.leans)
        self.line_lean.set_data(t, lean_arr)
        self.ax_lean.set_xlim(x_min, x_max)
        if len(lean_arr) > 10:
            lean_abs_max = max(np.max(np.abs(lean_arr)) + 3, 10)
            self.ax_lean.set_ylim(-lean_abs_max, lean_abs_max)

        # Panel 2: Slope
        slope_arr = np.array(self.slopes)
        self.line_slope.set_data(t, slope_arr)
        self.ax_slope.set_xlim(x_min, x_max)
        if len(slope_arr) > 10:
            slope_abs_max = max(np.max(np.abs(slope_arr)) + 3, 10)
            self.ax_slope.set_ylim(-slope_abs_max, slope_abs_max)

        # Panel 3: Vibration
        vib_arr = np.array(self.vibrations)
        self.line_vib.set_data(t, vib_arr)
        self.ax_vib.set_xlim(x_min, x_max)
        if len(vib_arr) > 10:
            vib_max = max(np.max(vib_arr) + 1, 3)
            self.ax_vib.set_ylim(0, vib_max)

        # Panel 4: Status dashboard
        cur_lean = lean_arr[-1]
        cur_slope = slope_arr[-1]
        cur_vib = vib_arr[-1]

        # Edge state
        if abs(cur_lean) < self.lean_flat_threshold_deg:
            edge_str = 'FLAT'
            edge_color = 'black'
        elif cur_lean > 0:
            edge_str = 'TOESIDE'
            edge_color = 'red'
        else:
            edge_str = 'HEELSIDE'
            edge_color = 'blue'

        self.status_texts['edge_val'].set_text(edge_str)
        self.status_texts['edge_val'].set_color(edge_color)

        self.status_texts['lean_val'].set_text(f'{cur_lean:+.1f} deg')
        self.status_texts['lean_val'].set_color(
            'red' if abs(cur_lean) > self.lean_warn_deg else 'blue')

        self.status_texts['slope_val'].set_text(f'{cur_slope:+.1f} deg')
        self.status_texts['slope_val'].set_color(
            'orange' if abs(cur_slope) > self.slope_warn_deg else 'green')

        self.status_texts['vib_val'].set_text(f'{cur_vib:.2f} m/s^2')

        # Stance
        self.status_texts['stance_val'].set_text('GOOFY' if self.is_goofy else 'REGULAR')

        # Data rate
        dt_total = t[-1] - t[0]
        rate = len(t) / dt_total if dt_total > 0 else 0
        self.status_texts['rate_val'].set_text(f'{rate:.0f} Hz')
        self.status_texts['samples_val'].set_text(f'{self.data_count}')
        self.status_texts['duration_val'].set_text(f'{t[-1]:.1f}s')

        # Title bar
        if self.cal_phase == CalPhase.PRE_CAL:
            elapsed = t[-1] - (self.phase_start_t or 0)
            remaining = max(0, self.pre_cal_delay - elapsed)
            self.fig.suptitle(
                f'Board Visualizer -- PLACE BOARD FLAT ({remaining:.1f}s)',
                fontsize=16, fontweight='bold', color='blue')
        elif self.cal_phase == CalPhase.FLAT_CAL:
            elapsed = t[-1] - (self.phase_start_t or 0)
            remaining = max(0, self.cal_duration - elapsed)
            self.fig.suptitle(
                f'Board Visualizer -- CALIBRATING (keep flat... {remaining:.1f}s)',
                fontsize=16, fontweight='bold', color='orange')
        else:
            self.fig.suptitle(
                f'Board Visualizer | {edge_str} {cur_lean:+.0f} deg | Slope {cur_slope:+.0f} deg',
                fontsize=16, fontweight='bold', color=edge_color)

        return ()


# ── Global instance ──────────────────────────────────────────────────────────

viz = BoardVisualizer()


# ── Server ───────────────────────────────────────────────────────────────────

def run_server():
    """Run asyncio ARVOS server in a background thread."""
    async def server_main():
        server = ArvosServer(host="0.0.0.0", port=8765)
        server.print_qr_code()

        def on_imu(data: IMUData):
            viz.add_data(data)

        async def on_connect(client_id: str):
            viz.is_connected = True
            print(f"Connected: {client_id}")
            print("Place board flat on the ground -- 3s delay then 3s calibration.")

        async def on_disconnect(client_id: str):
            viz.is_connected = False
            print(f"Disconnected: {client_id}")

        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect

        print("Starting board visualizer server on port 8765...")
        print("Connect your device, then place board flat for calibration.")
        print("Press 'g' in the plot window to toggle regular/goofy.\n")

        await server.start()

    asyncio.run(server_main())


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

        _time.sleep(1)

        ani = FuncAnimation(viz.fig, viz.update_plot, interval=50,
                           blit=False, cache_frame_data=False)
        plt.show()

    except KeyboardInterrupt:
        print("\n\nStopped")
