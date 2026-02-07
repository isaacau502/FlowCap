import Foundation
import CoreMotion

struct IMUData: Codable {
    let type: String = "imu"
    let timestampNs: Int64
    let linearAcceleration: [Double]
    let angularVelocity: [Double]
    let gravity: [Double]?
}

class IMUManager: ObservableObject {
    private let motionManager = CMMotionManager()
    private let queue = OperationQueue()
    
    @Published var acceleration: (x: Double, y: Double, z: Double)?
    @Published var gyro: (x: Double, y: Double, z: Double)?
    @Published var latestData: IMUData?
    
    init() {
        queue.maxConcurrentOperationCount = 1
        queue.qualityOfService = .userInitiated
    }
    
    func startIMU() {
        guard motionManager.isDeviceMotionAvailable else {
            print("Device motion not available")
            return
        }
        
        // Set update interval (100 Hz)
        motionManager.deviceMotionUpdateInterval = 0.01
        
        motionManager.startDeviceMotionUpdates(to: queue) { [weak self] (motion, error) in
            guard let motion = motion, error == nil else {
                print("Error getting motion data: \(error?.localizedDescription ?? "unknown")")
                return
            }
            
            let timestampNs = Int64(motion.timestamp * 1_000_000_000)
            
            let accel = motion.userAcceleration
            let gyro = motion.rotationRate
            let gravity = motion.gravity
            
            let imuData = IMUData(
                timestampNs: timestampNs,
                linearAcceleration: [accel.x, accel.y, accel.z],
                angularVelocity: [gyro.x, gyro.y, gyro.z],
                gravity: [gravity.x, gravity.y, gravity.z]
            )
            
            DispatchQueue.main.async {
                self?.acceleration = (accel.x, accel.y, accel.z)
                self?.gyro = (gyro.x, gyro.y, gyro.z)
                self?.latestData = imuData
            }
        }
        
        print("IMU started at 100 Hz")
    }
    
    func stopIMU() {
        motionManager.stopDeviceMotionUpdates()
        print("IMU stopped")
    }
}
