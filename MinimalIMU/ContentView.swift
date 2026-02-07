import SwiftUI

struct ContentView: View {
    @StateObject private var imuManager = IMUManager()
    @StateObject private var webSocketManager = WebSocketManager()
    
    @State private var serverIP = "172.26.6.110"
    @State private var serverPort = "8765"
    @State private var isStreaming = false
    
    var body: some View {
        VStack(spacing: 30) {
            Text("Minimal IMU Streamer")
                .font(.largeTitle)
                .fontWeight(.bold)
            
            // Connection Status
            HStack {
                Circle()
                    .fill(webSocketManager.isConnected ? Color.green : Color.red)
                    .frame(width: 12, height: 12)
                Text(webSocketManager.isConnected ? "Connected" : "Disconnected")
                    .foregroundColor(.gray)
            }
            
            // Server Configuration
            VStack(alignment: .leading, spacing: 10) {
                Text("Server IP:")
                    .font(.headline)
                TextField("IP Address", text: $serverIP)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .autocapitalization(.none)
                    .disabled(isStreaming)
                
                Text("Port:")
                    .font(.headline)
                TextField("Port", text: $serverPort)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .keyboardType(.numberPad)
                    .disabled(isStreaming)
            }
            .padding()
            .background(Color.gray.opacity(0.1))
            .cornerRadius(10)
            
            // IMU Data Display
            if isStreaming {
                VStack(alignment: .leading, spacing: 8) {
                    Text("IMU Data:")
                        .font(.headline)
                    
                    if let accel = imuManager.acceleration {
                        Text("Accel: (\(String(format: "%.2f", accel.x)), \(String(format: "%.2f", accel.y)), \(String(format: "%.2f", accel.z)))")
                            .font(.system(.body, design: .monospaced))
                    }
                    
                    if let gyro = imuManager.gyro {
                        Text("Gyro: (\(String(format: "%.2f", gyro.x)), \(String(format: "%.2f", gyro.y)), \(String(format: "%.2f", gyro.z)))")
                            .font(.system(.body, design: .monospaced))
                    }
                    
                    Text("Messages sent: \(webSocketManager.messagesSent)")
                        .font(.caption)
                        .foregroundColor(.gray)
                }
                .padding()
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(Color.blue.opacity(0.1))
                .cornerRadius(10)
            }
            
            Spacer()
            
            // Start/Stop Button
            Button(action: toggleStreaming) {
                Text(isStreaming ? "STOP STREAMING" : "START STREAMING")
                    .font(.title2)
                    .fontWeight(.bold)
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(isStreaming ? Color.red : Color.green)
                    .cornerRadius(15)
            }
        }
        .padding()
        .onChange(of: imuManager.latestData) { _, data in
            if let data = data, isStreaming {
                webSocketManager.sendIMUData(data)
            }
        }
    }
    
    private func toggleStreaming() {
        if isStreaming {
            // Stop streaming
            imuManager.stopIMU()
            webSocketManager.disconnect()
            isStreaming = false
        } else {
            // Start streaming
            let urlString = "ws://\(serverIP):\(serverPort)"
            webSocketManager.connect(to: urlString) { success in
                if success {
                    imuManager.startIMU()
                    isStreaming = true
                }
            }
        }
    }
}

#Preview {
    ContentView()
}
