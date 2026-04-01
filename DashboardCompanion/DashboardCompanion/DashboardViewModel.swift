import Foundation
import SwiftUI

@MainActor
final class DashboardViewModel: ObservableObject {
    @AppStorage("robotIP") var robotIP = "10.22.25.2"

    @Published var isConnected = false
    @Published var connectionSummary = "Waiting for robot"
    @Published var topicCount = 0
    @Published var valueCount = 0

    @Published var flywheelRPM: Double?
    @Published var flywheelRawRPM: Double?
    @Published var flywheelSetpoint: Double?
    @Published var flywheelError: Double?
    @Published var flywheelDuty: Double?
    @Published var flywheelAtSpeed: Bool?

    @Published var turretAngleDeg: Double?
    @Published var turretTargetDeg: Double?
    @Published var turretCRTStatus: String?
    @Published var turretCRTSeedDeg: Double?
    @Published var turretCRTErrorRot: Double?
    @Published var turretCRTSpread1: Double?
    @Published var turretCRTSpread2: Double?

    @Published var sotmStatus: String?
    @Published var sotmDistance: Double?
    @Published var sotmTurretDeg: Double?
    @Published var sotmFlywheelRPM: Double?
    @Published var sotmInRange: Bool?
    @Published var sotmActive: Bool?

    @Published var intakeAngleDeg: Double?
    @Published var intakeTargetDeg: Double?
    @Published var intakeErrorDeg: Double?
    @Published var intakeCurrent: Double?
    @Published var intakeRollerDuty: Double?

    @Published var loaderDuty: Double?
    @Published var loaderAutoReversing: Bool?

    private let client = NT4Client()
    private var started = false

    init() {
        client.onConnectionChanged = { [weak self] connected, summary in
            Task { @MainActor in
                self?.isConnected = connected
                self?.connectionSummary = summary
            }
        }

        client.onSnapshot = { [weak self] snapshot in
            Task { @MainActor in
                self?.apply(snapshot: snapshot)
            }
        }
    }

    func startIfNeeded() {
        guard !started else { return }
        started = true
        connect()
    }

    func connect() {
        client.connect(to: robotIP)
    }

    func refreshConnection() {
        client.connect(to: robotIP)
    }

    private func apply(snapshot: NT4Snapshot) {
        topicCount = snapshot.topicCount
        valueCount = snapshot.valueCount

        flywheelRPM = snapshot.doubleValue("Flywheel/RPM")
        flywheelRawRPM = snapshot.doubleValue("Flywheel/RawLeaderEncoderRPM")
        flywheelSetpoint = snapshot.doubleValue("Flywheel/SetpointRPM")
        flywheelError = snapshot.doubleValue("Flywheel/ErrorRPM")
        flywheelDuty = snapshot.doubleValue("Flywheel/AppliedDutyCycle")
        flywheelAtSpeed = snapshot.boolValue("Flywheel/AtSpeed")

        turretAngleDeg = snapshot.doubleValue("Turret/AngleDeg")
        turretTargetDeg = snapshot.doubleValue("Turret/TargetAngleDeg")
        turretCRTStatus = snapshot.stringValue("Turret/CRTStatus")
        turretCRTSeedDeg = snapshot.doubleValue("Turret/CRTSeedAngleDeg")
        turretCRTErrorRot = snapshot.doubleValue("Turret/CRTErrorRot")
        turretCRTSpread1 = snapshot.doubleValue("Turret/CRTEncoder1SpreadRot")
        turretCRTSpread2 = snapshot.doubleValue("Turret/CRTEncoder2SpreadRot")

        sotmStatus = snapshot.stringValue("ShootOTM/Status")
        sotmDistance = snapshot.doubleValue("ShootOTM/DistanceToTarget")
        sotmTurretDeg = snapshot.doubleValue("ShootOTM/TurretAngleDeg")
        sotmFlywheelRPM = snapshot.doubleValue("ShootOTM/FlywheelRPM")
        sotmInRange = snapshot.boolValue("ShootOTM/InRange")
        sotmActive = snapshot.boolValue("ShootOTM/Active")

        intakeAngleDeg = snapshot.doubleValue("Intake/PivotAngleDeg")
        intakeTargetDeg = snapshot.doubleValue("Intake/TargetAngleDeg")
        intakeErrorDeg = snapshot.doubleValue("Intake/PivotErrorDeg")
        intakeCurrent = snapshot.doubleValue("Intake/PivotCurrentAmps")
        intakeRollerDuty = snapshot.doubleValue("Intake/RollerDutyCycle")

        loaderDuty = snapshot.doubleValue("Loader/DutyCycle")
        loaderAutoReversing = snapshot.boolValue("Loader/AutoReversing")
    }

    func displayRPM(_ value: Double?) -> String {
        guard let value else { return "---" }
        return "\(Int(value.rounded())) rpm"
    }

    func displayAngle(_ value: Double?) -> String {
        guard let value else { return "---" }
        return String(format: "%.1f°", value)
    }

    func displayDuty(_ value: Double?) -> String {
        guard let value else { return "---" }
        return String(format: "%.2f", value)
    }

    func displayMeters(_ value: Double?) -> String {
        guard let value else { return "---" }
        return String(format: "%.2f m", value)
    }

    func displayAmps(_ value: Double?) -> String {
        guard let value else { return "---" }
        return String(format: "%.1f A", value)
    }

    func displayRotations(_ value: Double?) -> String {
        guard let value else { return "---" }
        return String(format: "%.4f rot", value)
    }

    func displayCRTSpread(_ first: Double?, _ second: Double?) -> String {
        guard let first, let second else { return "---" }
        return String(format: "%.4f / %.4f", first, second)
    }

    func booleanText(_ value: Bool?) -> String {
        guard let value else { return "---" }
        return value ? "YES" : "NO"
    }
}
