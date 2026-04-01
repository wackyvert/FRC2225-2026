import Foundation

struct NT4Snapshot {
    let values: [String: NT4Value]
    let topicCount: Int
    let valueCount: Int

    func doubleValue(_ key: String) -> Double? {
        guard case let .double(value)? = values["/SmartDashboard/" + key] else { return nil }
        return value
    }

    func boolValue(_ key: String) -> Bool? {
        guard case let .bool(value)? = values["/SmartDashboard/" + key] else { return nil }
        return value
    }

    func stringValue(_ key: String) -> String? {
        guard case let .string(value)? = values["/SmartDashboard/" + key] else { return nil }
        return value
    }
}

enum NT4Value {
    case double(Double)
    case bool(Bool)
    case string(String)
    case array([NT4Value])
    case null
}

final class NT4Client {
    var onConnectionChanged: ((Bool, String) -> Void)?
    var onSnapshot: ((NT4Snapshot) -> Void)?

    private let processingQueue = DispatchQueue(label: "frc2225.dashboard.nt4.processing", qos: .userInitiated)

    private var webSocketTask: URLSessionWebSocketTask?
    private var session: URLSession?
    private var reconnectWorkItem: DispatchWorkItem?
    private var shouldReconnect = true
    private var currentIP = "10.22.25.2"
    private var connectionGeneration = 0

    private var topicMap: [Int: String] = [:]
    private var valueMap: [String: NT4Value] = [:]
    private var subUID = 1
    private var latestSnapshot: NT4Snapshot?
    private var snapshotDispatchWorkItem: DispatchWorkItem?

    func connect(to ip: String) {
        currentIP = ip.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !currentIP.isEmpty else { return }

        disconnect(reconnect: false)
        shouldReconnect = true
        connectionGeneration += 1
        let generation = connectionGeneration

        guard let url = URL(string: "ws://\(currentIP):5810/nt/dashboard") else {
            onConnectionChanged?(false, "Bad IP")
            return
        }

        let config = URLSessionConfiguration.default
        config.waitsForConnectivity = false
        let session = URLSession(configuration: config)
        self.session = session
        let task = session.webSocketTask(with: url, protocols: ["networktables.first.wpi.edu"])
        self.webSocketTask = task
        task.resume()

        onConnectionChanged?(false, "Connecting to \(currentIP)")
        sendSubscriptions()
        listen(generation: generation)
    }

    func disconnect(reconnect: Bool = false) {
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        shouldReconnect = reconnect

        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        session?.invalidateAndCancel()
        session = nil
    }

    private func sendSubscriptions() {
        let payload: [[String: Any]] = [
            [
                "method": "subscribe",
                "params": [
                    "topics": ["/SmartDashboard/"],
                    "subuid": subUID,
                    "options": [
                        "all": false,
                        "topicsonly": false,
                        "prefix": true,
                        "periodic": 0.1
                    ]
                ]
            ],
            [
                "method": "subscribe",
                "params": [
                    "topics": ["/Field/", "/swerve/", "/VisionSystemSim-"],
                    "subuid": subUID + 1,
                    "options": [
                        "all": false,
                        "topicsonly": false,
                        "prefix": true,
                        "periodic": 0.1
                    ]
                ]
            ]
        ]
        subUID += 2

        guard let data = try? JSONSerialization.data(withJSONObject: payload),
              let text = String(data: data, encoding: .utf8)
        else { return }

        webSocketTask?.send(.string(text)) { _ in }
    }

    private func listen(generation: Int) {
        webSocketTask?.receive { [weak self] result in
            guard let self else { return }
            guard generation == self.connectionGeneration else { return }

            switch result {
            case let .success(message):
                self.onConnectionChanged?(true, "Connected to \(self.currentIP)")
                self.handle(message: message)
                self.listen(generation: generation)
            case let .failure(error):
                self.onConnectionChanged?(false, "Disconnected")
                self.scheduleReconnect()
                print("NT4 socket error: \(error)")
            }
        }
    }

    private func handle(message: URLSessionWebSocketTask.Message) {
        processingQueue.async { [weak self] in
            guard let self else { return }
            switch message {
            case let .string(text):
                self.handleText(text)
            case let .data(data):
                self.handleBinary(data)
            @unknown default:
                break
            }
        }
    }

    private func handleText(_ text: String) {
        guard let data = text.data(using: .utf8),
              let json = try? JSONSerialization.jsonObject(with: data) as? [[String: Any]]
        else { return }

        for message in json {
            guard let method = message["method"] as? String,
                  let params = message["params"] as? [String: Any]
            else { continue }

            switch method {
            case "announce":
                if let id = params["id"] as? Int,
                   let name = params["name"] as? String {
                    topicMap[id] = name
                }
            case "unannounce":
                if let id = params["id"] as? Int {
                    topicMap.removeValue(forKey: id)
                }
            default:
                break
            }
        }

        pushSnapshot()
    }

    private func handleBinary(_ data: Data) {
        let values = MessagePackDecoder.decodeTopLevelValues(data)
        for value in values {
            guard case let .array(items) = value, items.count >= 4 else { continue }
            guard case let .double(topicIDDouble) = items[0] else { continue }
            let topicID = Int(topicIDDouble)
            guard let topicName = topicMap[topicID] else { continue }
            valueMap[topicName] = convert(items[3])
        }
        pushSnapshot()
    }

    private func convert(_ value: MessagePackValue) -> NT4Value {
        switch value {
        case let .double(number):
            return .double(number)
        case let .bool(flag):
            return .bool(flag)
        case let .string(text):
            return .string(text)
        case let .array(values):
            return .array(values.map(convert))
        case .map:
            return .null
        case .null:
            return .null
        }
    }

    private func pushSnapshot() {
        latestSnapshot = NT4Snapshot(values: valueMap, topicCount: topicMap.count, valueCount: valueMap.count)
        guard snapshotDispatchWorkItem == nil else { return }

        let workItem = DispatchWorkItem { [weak self] in
            guard let self, let snapshot = self.latestSnapshot else { return }
            self.snapshotDispatchWorkItem = nil
            DispatchQueue.main.async {
                self.onSnapshot?(snapshot)
            }
        }

        snapshotDispatchWorkItem = workItem
        processingQueue.asyncAfter(deadline: .now() + 0.1, execute: workItem)
    }

    private func scheduleReconnect() {
        guard shouldReconnect else { return }
        reconnectWorkItem?.cancel()
        let workItem = DispatchWorkItem { [weak self] in
            guard let self else { return }
            self.connect(to: self.currentIP)
        }
        reconnectWorkItem = workItem
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0, execute: workItem)
    }
}
