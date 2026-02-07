#!/bin/bash
# Setup script for Minimal IMU Xcode project
# Run this on your Mac after transferring the files

echo "🚀 Setting up Minimal IMU Xcode Project..."
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if Xcode is installed
if ! command -v xcodebuild &> /dev/null; then
    echo "❌ Xcode not found. Please install Xcode from the App Store."
    exit 1
fi

echo "✅ Xcode found"
echo ""

echo "📝 Instructions:"
echo ""
echo "1. Open Xcode"
echo "2. File → New → Project"
echo "3. Choose 'App' template"
echo "4. Configure:"
echo "   - Product Name: MinimalIMU"
echo "   - Interface: SwiftUI"
echo "   - Language: Swift"
echo "   - Click Next"
echo ""
echo "5. Save it in this directory: $SCRIPT_DIR"
echo ""
echo "6. After project is created:"
echo "   - Delete the default ContentView.swift and MinimalIMUApp.swift"
echo "   - Drag these 4 files into your project:"
echo "     ✓ MinimalIMUApp.swift"
echo "     ✓ ContentView.swift"
echo "     ✓ IMUManager.swift"
echo "     ✓ WebSocketManager.swift"
echo ""
echo "7. Add Privacy Permission:"
echo "   - Click project name → Target → Info tab"
echo "   - Click + to add:"
echo "     Key: Privacy - Motion Usage Description"
echo "     Value: We need motion data to stream IMU sensors"
echo ""
echo "8. Connect your iPhone and press ▶️ to build!"
echo ""
echo "📁 Files ready in: $SCRIPT_DIR"
ls -la *.swift 2>/dev/null | grep -v "setup"
echo ""
echo "✨ You're all set! Open Xcode and follow the steps above."
