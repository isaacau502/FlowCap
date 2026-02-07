#!/usr/bin/env python3
"""
Live 3JS Bridge
Headless dual-IMU processor that broadcasts pose state to the browser via WebSocket.

  Thread 1: Board ArvosServer (port 9090)
  Thread 2: Body ArvosServer  (port 9091)
  Thread 3: WebSocket broadcast server (port 9093) -> browser 3JS sandbox
  Main:     Console input loop (c=calibrate, q=quit)

Signal processing matches live_dual_visualizer.py exactly.
Loads calibration_profile.json if available to skip manual calibration.
"""

import asyncio
import sys
import os
import json
import re
import time as _time
from pathlib import Path
from datetime import datetime
import numpy as np
from collections import deque
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))
from arvos import ArvosServer, IMUData

try:
    import websockets
except ImportError:
    # websockets is already available (used by Arvos SDK)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib', 'Arvos-sdk', 'python'))
    import websockets


# ── Complementary Filter (verbatim from live_dual_visualizer.py) ────────────

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


# ── Bridge State ────────────────────────────────────────────────────────────

class BridgeState:
    def __init__(self):
        # Board state
        self.board_connected = False
        self.board_last_ts_ns = None
        self.board_start_ns = None
        self.board_vib_raw = deque(maxlen=50)
        self.is_goofy = False

        # Board calibration (Rodrigues rotation)
        self.board_ref_gravity = None
        self.board_cal_gravities = []
        self.board_cal_start = None
        self.board_waiting_for_start = True
        self.board_pre_cal = False
        self.board_calibrating = False
        self.board_calibrated = False
        self.board_cal_rotation = np.eye(3)
        self.pre_cal_delay = 3.0
        self.cal_duration = 3.0

        # Body state
        self.body_connected = False
        self.body_last_ts_ns = None
        self.body_start_ns = None
        self.ori_filter = IMUOrientationFilter(alpha=0.6)

        # Body calibration
        self.body_ref_gravity = None
        self.body_cal_gravities = []
        self.body_cal_start = None
        self.body_waiting_for_start = True
        self.body_pre_cal = False
        self.body_calibrating = False
        self.body_calibrated = False
        self.max_tilt_deg = 45.0

        # Auto-recal: when loaded from profile, recalibrate on first body connect
        self.body_auto_recal = False
        self.body_auto_recal_samples = []
        self.body_auto_recal_start = None
        self.body_auto_recal_duration = 2.0  # seconds

        # Yaw drift correction
        self.yaw_baseline = deque(maxlen=500)

        # Calibration profile (loaded from JSON)
        self.cal_profile = None

        # Output state (sent to browser)
        self.current_state = {
            "squatPct": 0.0,
            "leanDeg": 0.0,
            "torsoRot": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "slopeDeg": 0.0,
            "boardAccelFwd": 0.0,
        }

        # Thread safety for current_state
        self.lock = threading.Lock()

    # ── Rodrigues rotation (verbatim from live_dual_visualizer.py) ────────

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
        slope_deg = -np.degrees(np.arctan2(g_cal[0], -g_cal[2]))
        if self.is_goofy:
            slope_deg = -slope_deg
        return lean_deg, slope_deg

    # ── Board data processing (verbatim from live_dual_visualizer.py) ─────

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

        # Board calibration state machine
        lean_deg = 0.0
        slope_deg = 0.0

        if self.board_waiting_for_start:
            lean_deg = np.degrees(np.arctan2(gy, -gz))

        elif self.board_pre_cal:
            if self.board_cal_start is None:
                self.board_cal_start = t
            if t - self.board_cal_start >= self.pre_cal_delay:
                self.board_pre_cal = False
                self.board_calibrating = True
                self.board_cal_start = t
                print("Board: CALIBRATING -- keep flat and still!")
            lean_deg = np.degrees(np.arctan2(gy, -gz))

        elif self.board_calibrating:
            self.board_cal_gravities.append([gx, gy, gz])
            if t - self.board_cal_start >= self.cal_duration:
                gravs = np.array(self.board_cal_gravities)
                self.board_ref_gravity = np.median(gravs, axis=0)
                self._compute_board_cal_rotation()
                self.board_calibrating = False
                self.board_calibrated = True
                g_mag = np.linalg.norm(self.board_ref_gravity)
                print(f"Board calibration complete! |g| = {g_mag:.3f} m/s^2")
            lean_deg = np.degrees(np.arctan2(gy, -gz))

        elif self.board_calibrated:
            lean_deg, slope_deg = self._decompose_gravity(gx, gy, gz)

        # Forward acceleration from board IMU (gravity-free)
        board_accel_fwd = 0.0
        if self.board_calibrated and data.linear_acceleration is not None:
            accel_raw = np.array(data.linear_acceleration)
            accel_cal = self.board_cal_rotation @ accel_raw
            board_accel_fwd = float(accel_cal[0])  # X = forward axis (slope direction)

        with self.lock:
            self.current_state["leanDeg"] = round(lean_deg, 2)
            self.current_state["slopeDeg"] = round(slope_deg, 2)
            self.current_state["boardAccelFwd"] = round(board_accel_fwd, 3)

    # ── Body data processing (verbatim from live_dual_visualizer.py) ──────

    def add_body_data(self, data: IMUData):
        if self.body_last_ts_ns is None:
            self.body_start_ns = data.timestamp_ns
            dt = 0.01
            if data.gravity is not None:
                gx, gy, gz = data.gravity
            else:
                gx, gy, gz = data.linear_acceleration
            print(f"  Body first gravity: [{gx:.3f}, {gy:.3f}, {gz:.3f}]  |g|={np.linalg.norm([gx,gy,gz]):.3f}")
            if self.body_ref_gravity is not None:
                print(f"  Body ref gravity:   [{self.body_ref_gravity[0]:.3f}, {self.body_ref_gravity[1]:.3f}, {self.body_ref_gravity[2]:.3f}]  |g|={np.linalg.norm(self.body_ref_gravity):.3f}")
                print(f"  Body calibrated: {self.body_calibrated}, max_tilt_deg: {self.max_tilt_deg}")
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

        # Calibration state machine
        squat_depth = 0.0
        torso_rotation = 0.0

        if self.body_waiting_for_start:
            pass

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
            # Auto-recal: collect first N seconds of gravity to adapt to current pocket orientation
            if self.body_auto_recal:
                if self.body_auto_recal_start is None:
                    self.body_auto_recal_start = t
                    print(f"Body auto-recalibrating for {self.body_auto_recal_duration}s — stand still...")
                self.body_auto_recal_samples.append([gx, gy, gz])
                if t - self.body_auto_recal_start >= self.body_auto_recal_duration:
                    gravs = np.array(self.body_auto_recal_samples)
                    self.body_ref_gravity = np.median(gravs, axis=0)
                    self.body_auto_recal = False
                    self.body_auto_recal_samples = []
                    mag = np.linalg.norm(self.body_ref_gravity)
                    print(f"Body auto-recal done! ref=[{self.body_ref_gravity[0]:.3f}, {self.body_ref_gravity[1]:.3f}, {self.body_ref_gravity[2]:.3f}] |g|={mag:.3f}")
                # Output 0% squat during auto-recal
            else:
                # Squat depth from gravity-vector tilt angle
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

        with self.lock:
            self.current_state["squatPct"] = round(squat_depth, 1)
            self.current_state["torsoRot"] = round(torso_rotation, 1)
            self.current_state["pitch"] = round(pitch, 1)
            self.current_state["roll"] = round(-roll, 1)

    # ── Calibration trigger ───────────────────────────────────────────────

    def start_calibration(self):
        """Reset both board and body calibration state machines."""
        self.board_ref_gravity = None
        self.board_cal_gravities = []
        self.board_cal_start = None
        self.board_waiting_for_start = False
        self.board_pre_cal = True
        self.board_calibrating = False
        self.board_calibrated = False
        self.board_cal_rotation = np.eye(3)

        self.body_ref_gravity = None
        self.body_cal_gravities = []
        self.body_cal_start = None
        self.body_waiting_for_start = False
        self.body_pre_cal = True
        self.body_calibrating = False
        self.body_calibrated = False
        self.body_auto_recal = False
        self.body_auto_recal_samples = []
        self.body_auto_recal_start = None
        self.yaw_baseline.clear()
        self.ori_filter = IMUOrientationFilter(alpha=0.6)

        print("\nCalibration started! Board flat + stand still. 3s delay + 3s capture...")


# ── Calibration Profile Loading ─────────────────────────────────────────────

def load_calibration_profile(state: BridgeState, path: str):
    """Load calibration_profile.json and pre-configure state."""
    try:
        with open(path, 'r') as f:
            profile = json.load(f)
        state.cal_profile = profile

        dr = profile.get("dynamicRanges", {})

        # Body reference gravity -> skip body calibration
        body_ref = dr.get("body_ref_gravity")
        if body_ref:
            state.body_ref_gravity = np.array(body_ref)
            state.body_waiting_for_start = False
            state.body_calibrated = True
            state.body_auto_recal = True  # recal on first body connect
            print(f"  Body ref gravity loaded: [{body_ref[0]:.3f}, {body_ref[1]:.3f}, {body_ref[2]:.3f}]")
            print(f"  (will auto-recalibrate on body connect for {state.body_auto_recal_duration}s)")

        # Board reference gravity -> compute Rodrigues rotation
        board_ref = dr.get("board_ref_gravity")
        if board_ref:
            state.board_ref_gravity = np.array(board_ref)
            state._compute_board_cal_rotation()
            state.board_waiting_for_start = False
            state.board_calibrated = True
            print(f"  Board ref gravity loaded: [{board_ref[0]:.3f}, {board_ref[1]:.3f}, {board_ref[2]:.3f}]")

        # Dynamic ranges
        if "maxSquatTiltDeg" in dr:
            state.max_tilt_deg = dr["maxSquatTiltDeg"]
            print(f"  Max squat tilt: {state.max_tilt_deg:.1f} deg")

        print("Calibration profile loaded successfully.")

    except Exception as e:
        print(f"Warning: Could not load calibration profile: {e}")


def build_config_message(state: BridgeState) -> dict:
    """Build the config message sent to browser on connect."""
    msg = {"type": "config"}
    if state.cal_profile:
        dims = state.cal_profile.get("bodyDims", {})
        dr = state.cal_profile.get("dynamicRanges", {})

        # Use anatomical inseam for hip height (not sensor pocket height)
        inseam_m = dims.get("inseamCm", 88.0) / 100.0
        sensor_standing = dims.get("standingHeight", 0.53)
        sensor_min = dr.get("minHeight", 0.239)
        squat_ratio = sensor_min / sensor_standing if sensor_standing > 0 else 0.45
        render_min = inseam_m * squat_ratio

        msg["dims"] = {
            "standingHeight": inseam_m,
            "torsoLength": dims.get("torsoLength"),
            "minHeight": render_min,
            "shinLength": inseam_m * 0.5,
            "upperArmLen": dims.get("upperArmLen"),
            "forearmLen": dims.get("forearmLen"),
            "shoulderSpread": dims.get("shoulderSpread"),
        }
        msg["ranges"] = {
            "maxLeanDeg": dr.get("maxLeanDeg"),
            "maxSquatTiltDeg": dr.get("maxSquatTiltDeg"),
            "maxTorsoRotDeg": dr.get("maxTorsoRotDeg"),
        }
    msg["calStatus"] = {
        "boardCalibrated": state.board_calibrated,
        "bodyCalibrated": state.body_calibrated,
    }
    return msg


# ── Session Recording Helpers ──────────────────────────────────────────────

RECORDINGS_DIR = Path(__file__).parent / "recordings"


def _sanitize_filename(name: str) -> str:
    """Strip unsafe characters from a session name for use in filenames."""
    return re.sub(r'[^a-zA-Z0-9_\- ]', '', name).strip().replace(' ', '_')[:60]


async def handle_save_session(websocket, cmd):
    """Save a session JSON to the recordings directory."""
    session = cmd.get("session")
    if not session or "frames" not in session:
        await websocket.send(json.dumps({"type": "session_error", "error": "No session data"}))
        return

    meta = session.get("meta", {})
    name = _sanitize_filename(meta.get("name", "untitled"))
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{ts}_{name}.json" if name else f"{ts}.json"

    RECORDINGS_DIR.mkdir(exist_ok=True)
    filepath = RECORDINGS_DIR / filename
    with open(filepath, 'w') as f:
        json.dump(session, f)

    print(f"Session saved: {filename} ({meta.get('frameCount', 0)} frames)")
    await websocket.send(json.dumps({"type": "session_saved", "id": filename}))


async def handle_list_sessions(websocket):
    """List all saved sessions (metadata only, no frames)."""
    RECORDINGS_DIR.mkdir(exist_ok=True)
    sessions = []
    for p in sorted(RECORDINGS_DIR.glob("*.json"), reverse=True):
        try:
            with open(p, 'r') as f:
                # Read only the first portion to extract meta without loading all frames
                data = json.load(f)
            sessions.append({
                "id": p.name,
                "meta": data.get("meta", {}),
                "config": data.get("config", {}),
            })
        except Exception:
            pass
    await websocket.send(json.dumps({"type": "session_list", "sessions": sessions}))


async def handle_load_session(websocket, cmd):
    """Load a full session file by id."""
    session_id = cmd.get("id", "")
    # Path traversal protection
    if '/' in session_id or '\\' in session_id or '..' in session_id:
        await websocket.send(json.dumps({"type": "session_error", "error": "Invalid id"}))
        return

    filepath = RECORDINGS_DIR / session_id
    if not filepath.exists():
        await websocket.send(json.dumps({"type": "session_error", "error": "Not found"}))
        return

    with open(filepath, 'r') as f:
        session = json.load(f)
    await websocket.send(json.dumps({"type": "session_data", "id": session_id, "session": session}))


async def handle_delete_session(websocket, cmd):
    """Delete a session file by id."""
    session_id = cmd.get("id", "")
    if '/' in session_id or '\\' in session_id or '..' in session_id:
        await websocket.send(json.dumps({"type": "session_error", "error": "Invalid id"}))
        return

    filepath = RECORDINGS_DIR / session_id
    if filepath.exists():
        filepath.unlink()
        print(f"Session deleted: {session_id}")
    await websocket.send(json.dumps({"type": "session_deleted", "id": session_id}))


# ── Server Threads ──────────────────────────────────────────────────────────

def run_board_server(state: BridgeState):
    async def server_main():
        server = ArvosServer("0.0.0.0", 9090)

        def on_imu(data: IMUData):
            state.add_board_data(data)

        async def on_connect(client_id):
            state.board_connected = True
            print(f"Board connected: {client_id}")

        async def on_disconnect(client_id):
            state.board_connected = False
            print(f"Board disconnected: {client_id}")

        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        print("Board server listening on port 9090...")
        await server.start()

    asyncio.run(server_main())


def run_body_server(state: BridgeState):
    async def server_main():
        server = ArvosServer("0.0.0.0", 9091)

        def on_imu(data: IMUData):
            state.add_body_data(data)

        async def on_connect(client_id):
            state.body_connected = True
            print(f"Body connected: {client_id}")

        async def on_disconnect(client_id):
            state.body_connected = False
            print(f"Body disconnected: {client_id}")

        server.on_imu = on_imu
        server.on_connect = on_connect
        server.on_disconnect = on_disconnect
        print("Body server listening on port 9091...")
        await server.start()

    asyncio.run(server_main())


def run_ws_broadcast(state: BridgeState):
    async def server_main():
        clients = set()

        async def handler(websocket):
            clients.add(websocket)
            # Send config on connect
            config_msg = build_config_message(state)
            try:
                await websocket.send(json.dumps(config_msg))
            except Exception:
                pass
            try:
                async for message in websocket:
                    try:
                        cmd = json.loads(message)
                        action = cmd.get("action")
                        if action == "calibrate":
                            state.start_calibration()
                        elif action == "save_session":
                            await handle_save_session(websocket, cmd)
                        elif action == "list_sessions":
                            await handle_list_sessions(websocket)
                        elif action == "load_session":
                            await handle_load_session(websocket, cmd)
                        elif action == "delete_session":
                            await handle_delete_session(websocket, cmd)
                    except Exception:
                        pass
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                clients.discard(websocket)

        # Broadcast loop: send state at ~30Hz
        async def broadcast_loop():
            while True:
                if clients:
                    with state.lock:
                        msg = {
                            "type": "state",
                            "squatPct": state.current_state["squatPct"],
                            "leanDeg": state.current_state["leanDeg"],
                            "torsoRot": state.current_state["torsoRot"],
                            "pitch": state.current_state["pitch"],
                            "roll": state.current_state["roll"],
                            "slopeDeg": state.current_state["slopeDeg"],
                            "boardAccelFwd": state.current_state["boardAccelFwd"],
                            "boardConnected": state.board_connected,
                            "bodyConnected": state.body_connected,
                        }
                    payload = json.dumps(msg)
                    # Broadcast to all connected browsers
                    dead = set()
                    for ws in clients.copy():
                        try:
                            await ws.send(payload)
                        except Exception:
                            dead.add(ws)
                    clients.difference_update(dead)
                await asyncio.sleep(1 / 30)  # ~30 Hz

        async with websockets.serve(handler, "0.0.0.0", 9093):
            print("WebSocket broadcast server on port 9093...")
            await broadcast_loop()

    asyncio.run(server_main())


# ── Main ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("LIVE 3JS BRIDGE")
    print("=" * 60)
    print("Board IMU:  ws://<laptop-ip>:9090")
    print("Body IMU:   ws://<laptop-ip>:9091")
    print("Browser WS: ws://localhost:9093")
    print("=" * 60)

    state = BridgeState()

    # Load calibration profile if available
    profile_path = os.path.join(os.path.dirname(__file__), "calibration_profile.json")
    if os.path.exists(profile_path):
        print(f"\nLoading calibration profile: {profile_path}")
        load_calibration_profile(state, profile_path)
    else:
        print("\nNo calibration_profile.json found. Using defaults.")
        print("Press 'c' to start manual calibration once IMUs are connected.")

    print("\nCommands: c=calibrate  q=quit")
    print("-" * 60)

    try:
        board_thread = threading.Thread(target=run_board_server, args=(state,), daemon=True)
        body_thread = threading.Thread(target=run_body_server, args=(state,), daemon=True)
        ws_thread = threading.Thread(target=run_ws_broadcast, args=(state,), daemon=True)
        board_thread.start()
        body_thread.start()
        ws_thread.start()

        _time.sleep(0.5)

        # Console input loop
        while True:
            try:
                cmd = input().strip().lower()
                if cmd == 'c':
                    state.start_calibration()
                elif cmd == 'q':
                    print("Shutting down...")
                    break
                elif cmd == 'g':
                    state.is_goofy = not state.is_goofy
                    print(f"Goofy mode: {'ON' if state.is_goofy else 'OFF'}")
            except EOFError:
                # Non-interactive mode, just keep running
                _time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nStopped")
