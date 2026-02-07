# Setup on Mac

## Step 1: Transfer Files

Transfer the entire `MinimalIMU` folder to your Mac. You should have:
- ✅ `MinimalIMUApp.swift`
- ✅ `ContentView.swift`
- ✅ `IMUManager.swift`
- ✅ `WebSocketManager.swift`
- ✅ `README.md`
- ✅ This file

## Step 2: Create Xcode Project

1. **Open Xcode** on your Mac

2. **Create New Project:**
   ```
   File → New → Project
   ```

3. **Choose Template:**
   - Select "iOS" at the top
   - Choose "App"
   - Click "Next"

4. **Configure Project:**
   - Product Name: `MinimalIMU`
   - Team: (select your Apple ID or leave as None for local testing)
   - Organization Identifier: `com.yourname` (anything is fine)
   - Interface: **SwiftUI** ⚠️ IMPORTANT
   - Language: **Swift** ⚠️ IMPORTANT
   - Click "Next"

5. **Save Location:**
   - Choose the `MinimalIMU` folder location
   - Click "Create"

## Step 3: Add Your Files

1. **In Xcode, delete these default files:**
   - Right-click → Delete → "Move to Trash"
     - `ContentView.swift`
     - `MinimalIMUApp.swift`

2. **Add your files:**
   - Drag all 4 `.swift` files from Finder into Xcode's file list
   - Check "✓ Copy items if needed"
   - Check "✓ MinimalIMU" under "Add to targets"
   - Click "Finish"

## Step 4: Add Privacy Permission

1. Click on **project name** (blue icon) in file list
2. Select your **target** (should be "MinimalIMU")
3. Go to **"Info"** tab
4. Right-click in the list → "Add Row" (or click the + button)
5. Start typing "Motion" and select:
   ```
   Privacy - Motion Usage Description
   ```
6. For the value, enter:
   ```
   We need motion data to stream IMU sensors
   ```

## Step 5: Build and Run

1. **Connect your iPhone** via USB cable
2. **Unlock your iPhone**
3. **Select your iPhone** as the build target (top toolbar in Xcode)
4. **Click the Play button ▶️** or press `Cmd+R`

First time:
- You may need to trust your Mac on iPhone
- You may need to enable Developer Mode on iPhone (Settings → Privacy & Security → Developer Mode)
- You may need to trust your Apple ID (Settings → General → VPN & Device Management)

## Step 6: Use the App

1. Enter your PC's IP address: `172.26.6.110`
2. Enter port: `8765`
3. Make sure your Python server is running
4. Tap **"START STREAMING"**
5. Check your PC terminal - you should see IMU data!

## Troubleshooting

### "Signing for 'MinimalIMU' requires a development team"
- Click on project → Target → Signing & Capabilities
- Check "Automatically manage signing"
- Select your Apple ID in the "Team" dropdown
- (Or add your Apple ID: Xcode → Settings → Accounts → +)

### "Could not launch MinimalIMU"
- On iPhone: Settings → General → VPN & Device Management
- Trust your developer certificate

### No motion data
- Make sure you granted Motion & Fitness permission
- Check iPhone: Settings → Privacy & Security → Motion & Fitness → MinimalIMU (should be ON)

## Files Structure

```
MinimalIMU/
├── MinimalIMUApp.swift          # App entry point
├── ContentView.swift            # Main UI
├── IMUManager.swift             # IMU sensor handling
├── WebSocketManager.swift       # WebSocket connection
└── README.md                    # Documentation
```

## You're Done!

The app should now be streaming IMU data at 100 Hz to your Python server.
