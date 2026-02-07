#!/usr/bin/env python3
"""
Biomechanics Calibration Script
Collects real physical data from the rider to tune the 3D visualizer.

Phase A: Manual body dimensions (console input)
Phase B: IMU dynamic calibration (guided steps with live feedback)
Phase C: Free skate recording (optional, outdoor)

Outputs: calibration_profile.json
"""

import asyncio
import sys
import os
import time
import json
import numpy as np
from collections import deque
import threading
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))
from arvos import ArvosServer, IMUData


# ── Complementary Filter (from live_squat_visualizer.py) ────────────────────

class IMUOrientationFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initialized = False

    def update(self, gx, gy, gz, wx, wy, wz, dt):
        if dt <= 0 or dt > 1.0:
            dt = 0.01
        self.roll += wz * dt
        self.pitch += wx * dt
        self.yaw += wy * dt
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


# ── Rodrigues Rotation (from live_board_visualizer.py) ──────────────────────

def compute_cal_rotation(ref_gravity):
    g_mag = np.linalg.norm(ref_gravity)
    if g_mag < 0.01:
        return np.eye(3)
    g_unit = ref_gravity / g_mag
    target = np.array([0.0, 0.0, -1.0])
    axis = np.cross(g_unit, target)
    sin_a = np.linalg.norm(axis)
    cos_a = np.dot(g_unit, target)
    if sin_a < 1e-6:
        return np.eye(3) if cos_a > 0 else np.diag([-1, -1, 1])
    axis = axis / sin_a
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)


def decompose_gravity(gx, gy, gz, cal_rotation):
    g_cal = cal_rotation @ np.array([gx, gy, gz])
    lean_deg = np.degrees(np.arctan2(g_cal[1], -g_cal[2]))
    slope_deg = np.degrees(np.arctan2(g_cal[0], -g_cal[2]))
    return lean_deg, slope_deg


def gravity_tilt_angle(gx, gy, gz, ref_gravity):
    g_cur = np.array([gx, gy, gz])
    dot = np.dot(g_cur, ref_gravity)
    mag_cur = np.linalg.norm(g_cur)
    mag_ref = np.linalg.norm(ref_gravity)
    if mag_cur < 0.01 or mag_ref < 0.01:
        return 0.0
    cos_angle = np.clip(dot / (mag_cur * mag_ref), -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))


# ── Data Collection ─────────────────────────────────────────────────────────

class DataCollector:
    """Collects IMU data from both sensors."""

    def __init__(self):
        # Board data
        self.board_gravities = []
        self.board_linear_accels = []
        self.board_timestamps = []
        self.board_connected = False
        self.board_last_ts_ns = None

        # Body data
        self.body_gravities = []
        self.body_angular_vels = []
        self.body_timestamps = []
        self.body_connected = False
        self.body_last_ts_ns = None

        # Orientation filter
        self.ori_filter = IMUOrientationFilter(alpha=0.98)
        self.body_orientations = []  # (roll, pitch, yaw)

        self.recording = False

    def add_board_data(self, data: IMUData):
        if not self.recording:
            return
        g = data.gravity if data.gravity else [0, 0, 0]
        self.board_gravities.append(g)
        la = data.linear_acceleration if data.linear_acceleration else [0, 0, 0]
        self.board_linear_accels.append(list(la))
        self.board_timestamps.append(data.timestamp_ns)

    def add_body_data(self, data: IMUData):
        if not self.recording:
            return
        g = data.gravity if data.gravity else [0, 0, 0]
        gx, gy, gz = g
        av = data.angular_velocity if data.angular_velocity else [0, 0, 0]
        wx, wy, wz = av

        if self.body_last_ts_ns is None:
            dt = 0.01
        else:
            dt = (data.timestamp_ns - self.body_last_ts_ns) / 1e9
        self.body_last_ts_ns = data.timestamp_ns

        roll, pitch, yaw = self.ori_filter.update(gx, gy, gz, wx, wy, wz, dt)

        self.body_gravities.append(g)
        self.body_angular_vels.append([wx, wy, wz])
        self.body_timestamps.append(data.timestamp_ns)
        self.body_orientations.append([roll, pitch, yaw])

    def start_recording(self):
        self.board_gravities = []
        self.board_linear_accels = []
        self.board_timestamps = []
        self.body_gravities = []
        self.body_angular_vels = []
        self.body_timestamps = []
        self.body_orientations = []
        self.recording = True

    def stop_recording(self):
        self.recording = False
        return {
            "board_gravities": self.board_gravities,
            "board_accels": self.board_linear_accels,
            "body_gravities": self.body_gravities,
            "body_orientations": self.body_orientations,
            "board_count": len(self.board_gravities),
            "body_count": len(self.body_gravities)
        }


# ── Server Threads ──────────────────────────────────────────────────────────

collector = DataCollector()


def run_board_server():
    async def main():
        server = ArvosServer("0.0.0.0", 9090)
        server.on_imu = lambda data: collector.add_board_data(data)

        async def on_connect(cid):
            collector.board_connected = True
            print(f"  📟 Board connected: {cid}")

        async def on_disconnect(cid):
            collector.board_connected = False
            print(f"  📟 Board disconnected: {cid}")

        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        await server.start()

    asyncio.run(main())


def run_body_server():
    async def main():
        server = ArvosServer("0.0.0.0", 9091)
        server.on_imu = lambda data: collector.add_body_data(data)

        async def on_connect(cid):
            collector.body_connected = True
            print(f"  🧍 Body connected: {cid}")

        async def on_disconnect(cid):
            collector.body_connected = False
            print(f"  🧍 Body disconnected: {cid}")

        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        await server.start()

    asyncio.run(main())


# ── Calibration Routine ─────────────────────────────────────────────────────

def wait_for_enter(prompt):
    input(f"\n>>> {prompt} [Press ENTER when ready] ")


def record_for(seconds, label):
    """Record data for N seconds with countdown."""
    print(f"\n  Recording {label}...")
    collector.start_recording()
    for i in range(seconds, 0, -1):
        print(f"    {i}...", end=" ", flush=True)
        time.sleep(1)
    data = collector.stop_recording()
    print(f"DONE! ({data['board_count']} board, {data['body_count']} body samples)")
    return data


def phase_a_manual_dims():
    """Phase A: Manual body dimensions via console input."""
    print("\n" + "="*60)
    print("PHASE A: Body Dimensions (manual input)")
    print("="*60)
    print("Grab a tape measure. All measurements in centimeters.\n")

    height_cm = float(input("  Your height (cm): "))
    inseam_cm = float(input("  Floor to hip bone (cm): "))
    arm_cm = float(input("  Shoulder to wrist (cm): "))
    shoulder_cm = float(input("  Shoulder width (cm): "))

    # Normalize to unit height (the 3js visualizer uses normalized coords)
    scale = height_cm / 100.0  # Convert to meters

    standing_height = inseam_cm / 100.0 / scale
    torso_length = (height_cm - inseam_cm) / 100.0 / scale
    upper_arm = (arm_cm * 0.55) / 100.0 / scale
    forearm = (arm_cm * 0.45) / 100.0 / scale
    shoulder_spread = shoulder_cm / 100.0 / scale

    dims = {
        "heightCm": height_cm,
        "inseamCm": inseam_cm,
        "armLengthCm": arm_cm,
        "shoulderWidthCm": shoulder_cm,
        "standingHeight": round(standing_height, 3),
        "torsoLength": round(torso_length, 3),
        "upperArmLen": round(upper_arm, 3),
        "forearmLen": round(forearm, 3),
        "shoulderSpread": round(shoulder_spread, 3),
        "scale": round(scale, 3)
    }

    print(f"\n  Derived dimensions (normalized):")
    print(f"    Standing hip height: {dims['standingHeight']:.3f}")
    print(f"    Torso length:        {dims['torsoLength']:.3f}")
    print(f"    Upper arm:           {dims['upperArmLen']:.3f}")
    print(f"    Forearm:             {dims['forearmLen']:.3f}")
    print(f"    Shoulder spread:     {dims['shoulderSpread']:.3f}")
    print(f"    Scale factor:        {dims['scale']:.3f}")

    return dims


def phase_b_imu_calibration(dims):
    """Phase B: IMU dynamic calibration with guided steps."""
    print("\n" + "="*60)
    print("PHASE B: IMU Dynamic Calibration")
    print("="*60)

    results = {}

    # ── Step 1: Stand still (body reference gravity) ─────────────────
    wait_for_enter("STEP 1: Stand upright, still, with body IMU in pocket/backpack")
    data = record_for(3, "standing reference")

    if len(data["body_gravities"]) > 0:
        body_ref_gravity = np.median(data["body_gravities"], axis=0)
        results["body_ref_gravity"] = body_ref_gravity.tolist()
        print(f"  Body ref gravity: [{body_ref_gravity[0]:.3f}, {body_ref_gravity[1]:.3f}, {body_ref_gravity[2]:.3f}]")
        print(f"  |g| = {np.linalg.norm(body_ref_gravity):.3f} m/s²")
    else:
        print("  WARNING: No body data! Is body IMU connected?")
        body_ref_gravity = None

    # ── Step 2: Board flat (board reference gravity) ─────────────────
    wait_for_enter("STEP 2: Place board FLAT on ground with board IMU mounted")
    data = record_for(3, "board flat reference")

    if len(data["board_gravities"]) > 0:
        board_ref_gravity = np.median(data["board_gravities"], axis=0)
        board_cal_rotation = compute_cal_rotation(board_ref_gravity)
        results["board_ref_gravity"] = board_ref_gravity.tolist()
        print(f"  Board ref gravity: [{board_ref_gravity[0]:.3f}, {board_ref_gravity[1]:.3f}, {board_ref_gravity[2]:.3f}]")
        print(f"  |g| = {np.linalg.norm(board_ref_gravity):.3f} m/s² ({len(data['board_gravities'])} samples)")
    else:
        print("  WARNING: No board data! Is board IMU connected on port 9090?")
        board_ref_gravity = None
        board_cal_rotation = np.eye(3)

    # ── Step 3: Max squat ────────────────────────────────────────────
    wait_for_enter("STEP 3: Squat as DEEP as you would while skating. Hold it.")
    data = record_for(3, "max squat")

    if body_ref_gravity is not None and len(data["body_gravities"]) > 0:
        tilt_angles = [gravity_tilt_angle(*g, body_ref_gravity) for g in data["body_gravities"]]
        max_squat_tilt = np.percentile(tilt_angles, 90)  # 90th percentile for robustness
        # Cap tilt at 50 deg - beyond that is phone shifting, not actual squat
        max_squat_tilt = min(max_squat_tilt, 50.0)
        results["maxSquatTiltDeg"] = round(float(max_squat_tilt), 1)

        # Estimate minHeight as fraction of standingHeight
        # Map 0-50 deg tilt to 100%-45% of standing height (non-linear)
        # A 30 deg tilt ≈ typical deep skating squat ≈ 55% of standing
        squat_frac = (max_squat_tilt / 50.0)  # 0 to 1
        squat_ratio = max(0.45, 1.0 - squat_frac * 0.55)  # floor at 45% of standing
        min_height = dims["standingHeight"] * squat_ratio
        results["minHeight"] = round(float(min_height), 3)

        print(f"  Max squat tilt: {max_squat_tilt:.1f} deg (raw 90th pct, capped at 50)")
        print(f"  Squat ratio: {squat_ratio:.2f} of standing height")
        print(f"  Estimated minHeight: {min_height:.3f} (standing: {dims['standingHeight']:.3f})")
    else:
        print("  Skipped (no body data)")

    # ── Step 4: Max lean heelside ────────────────────────────────────
    wait_for_enter("STEP 4: Tilt board HEELSIDE as far as comfortable")
    data = record_for(3, "max heelside lean")

    max_heel = 0.0
    if len(data["board_gravities"]) > 0:
        leans = [decompose_gravity(*g, board_cal_rotation)[0] for g in data["board_gravities"]]
        max_heel = abs(np.percentile(leans, 10))  # Heelside is negative
        print(f"  Max heelside lean: {max_heel:.1f} deg")
        print(f"  ({len(data['board_gravities'])} samples, lean range: [{min(leans):.1f}, {max(leans):.1f}])")
    else:
        print("  WARNING: No board data! Is the board IMU connected on port 9090?")

    # ── Step 5: Max lean toeside ─────────────────────────────────────
    wait_for_enter("STEP 5: Tilt board TOESIDE as far as comfortable")
    data = record_for(3, "max toeside lean")

    max_toe = 0.0
    if len(data["board_gravities"]) > 0:
        leans = [decompose_gravity(*g, board_cal_rotation)[0] for g in data["board_gravities"]]
        max_toe = abs(np.percentile(leans, 90))  # Toeside is positive
        print(f"  Max toeside lean: {max_toe:.1f} deg")
        print(f"  ({len(data['board_gravities'])} samples, lean range: [{min(leans):.1f}, {max(leans):.1f}])")
    else:
        print("  WARNING: No board data! Is the board IMU connected on port 9090?")

    max_lean = max(max_heel, max_toe, 15.0)  # Floor at 15 deg
    results["maxLeanDeg"] = round(float(max_lean), 1)
    print(f"  --> maxLeanDeg: {max_lean:.1f} deg")

    # ── Step 6: Max torso rotation ───────────────────────────────────
    wait_for_enter("STEP 6: Twist your shoulders LEFT and RIGHT as far as you'd carve")

    # Reset orientation filter for clean yaw tracking
    collector.ori_filter = IMUOrientationFilter(alpha=0.98)
    data = record_for(5, "max torso rotation")

    if len(data["body_orientations"]) > 0:
        yaws = np.array([o[2] for o in data["body_orientations"]])

        # Remove gyro drift: subtract linear trend
        t_idx = np.arange(len(yaws))
        if len(yaws) > 2:
            slope, intercept = np.polyfit(t_idx, yaws, 1)
            yaws_detrended = yaws - (slope * t_idx + intercept)
            drift_rate = slope * 100  # drift per 100 samples
            print(f"  Yaw drift removed: {drift_rate:.1f} deg/100 samples")
        else:
            yaws_detrended = yaws

        yaw_range = float(np.max(yaws_detrended) - np.min(yaws_detrended))
        max_rot = yaw_range / 2  # Half the total range = max one-side rotation
        max_rot = min(max_rot, 60.0)  # Cap at 60 deg (realistic skating max)
        results["maxTorsoRotDeg"] = round(float(max_rot), 1)
        print(f"  Yaw range (detrended): {yaw_range:.1f} deg total")
        print(f"  --> maxTorsoRotDeg: {max_rot:.1f} deg (each direction)")
    else:
        print("  Skipped (no body data)")

    return results


def phase_c_free_skate():
    """Phase C: Optional free skate to capture natural ranges."""
    print("\n" + "="*60)
    print("PHASE C: Free Skate (optional)")
    print("="*60)

    ans = input("\n  Do you want to record a free skate session? (y/n): ").strip().lower()
    if ans != 'y':
        print("  Skipping free skate.")
        return None

    wait_for_enter("Skate naturally for 15 seconds - carve, squat, turn")
    data = record_for(15, "free skate")

    results = {}
    if len(data["board_gravities"]) > 0:
        # We need the board cal rotation - recalculate or pass it
        # For simplicity, use raw lean estimate
        leans = [np.degrees(np.arctan2(g[1], -g[2])) for g in data["board_gravities"]]
        results["leanRange"] = [round(float(np.min(leans)), 1), round(float(np.max(leans)), 1)]
        print(f"  Lean range: {results['leanRange']}")

    if len(data["body_orientations"]) > 0:
        yaws = np.array([o[2] for o in data["body_orientations"]])
        # Detrend to remove gyro drift
        if len(yaws) > 2:
            t_idx = np.arange(len(yaws))
            slope, intercept = np.polyfit(t_idx, yaws, 1)
            yaws = yaws - (slope * t_idx + intercept)
        results["rotRange"] = [round(float(np.min(yaws)), 1), round(float(np.max(yaws)), 1)]
        print(f"  Rotation range: {results['rotRange']}")

    return results


def save_profile(dims, dynamic, free_skate):
    """Save calibration profile to JSON."""
    profile = {
        "bodyDims": dims,
        "dynamicRanges": dynamic,
        "freeSkateRanges": free_skate if free_skate else {},
        "timestamp": datetime.now().isoformat()
    }

    path = os.path.join(os.path.dirname(__file__), "calibration_profile.json")
    with open(path, 'w') as f:
        json.dump(profile, f, indent=2)

    print(f"\n  Saved to: {path}")
    return path


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("="*60)
    print("BIOMECHANICS CALIBRATION")
    print("="*60)
    print("This script collects your body dimensions and movement")
    print("ranges to tune the 3D skateboard visualizer.")
    print()
    print("Board IMU: ws://<laptop-ip>:9090")
    print("Body IMU:  ws://<laptop-ip>:9091")
    print("="*60)

    # Start IMU servers
    board_thread = threading.Thread(target=run_board_server, daemon=True)
    body_thread = threading.Thread(target=run_body_server, daemon=True)
    board_thread.start()
    body_thread.start()

    time.sleep(1)
    print("\nServers started. Connect your IMUs before proceeding.")
    print(f"  Board: {'CONNECTED' if collector.board_connected else 'waiting...'}")
    print(f"  Body:  {'CONNECTED' if collector.body_connected else 'waiting...'}")

    wait_for_enter("Connect both IMUs, then press ENTER to begin")

    print(f"\n  Board: {'CONNECTED' if collector.board_connected else 'NOT CONNECTED'}")
    print(f"  Body:  {'CONNECTED' if collector.body_connected else 'NOT CONNECTED'}")

    # Phase A: Manual body dimensions
    dims = phase_a_manual_dims()

    # Phase B: IMU dynamic calibration
    dynamic = phase_b_imu_calibration(dims)

    # Phase C: Free skate (optional)
    free_skate = phase_c_free_skate()

    # Save profile
    print("\n" + "="*60)
    print("CALIBRATION COMPLETE")
    print("="*60)

    path = save_profile(dims, dynamic, free_skate)

    print(f"\n  Profile saved to: {path}")
    print(f"\n  Summary:")
    print(f"    Standing height: {dims.get('standingHeight', '?')}")
    print(f"    Min height:      {dynamic.get('minHeight', '?')}")
    print(f"    Max lean:        {dynamic.get('maxLeanDeg', '?')} deg")
    print(f"    Max squat tilt:  {dynamic.get('maxSquatTiltDeg', '?')} deg")
    print(f"    Max torso rot:   {dynamic.get('maxTorsoRotDeg', '?')} deg")
    print(f"\n  Load this in the 3js visualizer to use your calibrated profile!")
