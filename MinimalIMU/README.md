# Minimal IMU Streamer

Simple iOS app to stream IMU data to a WebSocket server.

## Setup in Xcode

1. **Create New Project:**
   - Open Xcode
   - File → New → Project
   - Choose "App" template
   - Product Name: `MinimalIMU`
   - Interface: SwiftUI
   - Language: Swift

2. **Add Files:**
   - Copy these 4 Swift files into your project:
     - `MinimalIMUApp.swift` (replace the default one)
     - `ContentView.swift` (replace the default one)
     - `IMUManager.swift` (add new)
     - `WebSocketManager.swift` (add new)

3. **Configure Info.plist:**
   - Click on your project in Xcode
   - Select your target
   - Go to "Info" tab
   - Add these privacy descriptions:
     - Key: `NSMotionUsageDescription`
     - Value: `We need motion data to stream IMU sensors`

4. **Build and Run:**
   - Connect your iPhone
   - Select it as the build target
   - Press Cmd+R to build and run

## Usage

1. Enter your PC's IP address (e.g., `172.26.6.110`)
2. Enter port (default: `8765`)
3. Tap "START STREAMING"
4. IMU data will stream at 100 Hz to your Python server

## Features

- ✅ 100 Hz IMU streaming
- ✅ Real-time data display
- ✅ Connection status indicator
- ✅ Message counter
- ✅ Clean, minimal UI

## Data Format

Sends JSON messages:
```json
{
  "type": "imu",
  "timestampNs": 1234567890000,
  "linearAcceleration": [0.1, -9.8, 0.0],
  "angularVelocity": [0.01, -0.02, 0.00],
  "gravity": [0.0, -1.0, 0.0]
}
```

## Compatible with

Your existing Python server at `minimal_imu.py` - it will receive and print the data!
