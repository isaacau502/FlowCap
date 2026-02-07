import Foundation

class WebSocketManager: NSObject, ObservableObject, URLSessionWebSocketDelegate {
    @Published var isConnected = false
    @Published var messagesSent = 0
    
    private var webSocketTask: URLSessionWebSocketTask?
    private var urlSession: URLSession?
    private let encoder = JSONEncoder()
    
    override init() {
        super.init()
        urlSession = URLSession(configuration: .default, delegate: self, delegateQueue: nil)
    }
    
    func connect(to urlString: String, completion: @escaping (Bool) -> Void) {
        guard let url = URL(string: urlString) else {
            print("Invalid URL: \(urlString)")
            completion(false)
            return
        }
        
        webSocketTask = urlSession?.webSocketTask(with: url)
        webSocketTask?.resume()
        
        // Start receiving messages
        receiveMessage()
        
        // Wait a moment for connection
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
            self?.isConnected = true
            completion(true)
            print("Connected to \(urlString)")
        }
    }
    
    func disconnect() {
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        isConnected = false
        messagesSent = 0
        print("Disconnected")
    }
    
    func sendIMUData(_ data: IMUData) {
        guard isConnected else { return }
        
        do {
            let jsonData = try encoder.encode(data)
            let message = URLSessionWebSocketTask.Message.string(String(data: jsonData, encoding: .utf8)!)
            
            webSocketTask?.send(message) { [weak self] error in
                if let error = error {
                    print("WebSocket send error: \(error)")
                } else {
                    DispatchQueue.main.async {
                        self?.messagesSent += 1
                    }
                }
            }
        } catch {
            print("JSON encoding error: \(error)")
        }
    }
    
    private func receiveMessage() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .success(let message):
                switch message {
                case .string(let text):
                    print("Received text: \(text)")
                case .data(let data):
                    print("Received data: \(data.count) bytes")
                @unknown default:
                    break
                }
                // Continue receiving
                self?.receiveMessage()
                
            case .failure(let error):
                print("Receive error: \(error)")
            }
        }
    }
    
    // URLSessionWebSocketDelegate
    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didOpenWithProtocol protocol: String?) {
        DispatchQueue.main.async {
            self.isConnected = true
        }
        print("WebSocket connected")
    }
    
    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didCloseWith closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        DispatchQueue.main.async {
            self.isConnected = false
        }
        print("WebSocket closed")
    }
}
