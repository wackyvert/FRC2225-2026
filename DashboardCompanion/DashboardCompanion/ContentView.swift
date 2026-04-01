import SwiftUI

struct ContentView: View {
    @StateObject private var model = DashboardViewModel()
    @FocusState private var ipFieldFocused: Bool

    var body: some View {
        ZStack {
            LinearGradient(
                colors: [
                    Color(red: 0.05, green: 0.06, blue: 0.09),
                    Color(red: 0.02, green: 0.03, blue: 0.05)
                ],
                startPoint: .topLeading,
                endPoint: .bottomTrailing
            )
            .ignoresSafeArea()

            ScrollView {
                VStack(spacing: 14) {
                    topBar
                    headerCard
                    quickStats
                    telemetryGrid
                }
                .frame(maxWidth: .infinity, alignment: .top)
                .padding(.horizontal, 14)
                .padding(.top, 8)
                .padding(.bottom, 16)
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
            .safeAreaPadding(.top, 18)
            .safeAreaPadding(.bottom, 8)
            .scrollDismissesKeyboard(.interactively)
        }
        .ignoresSafeArea()
        .preferredColorScheme(.dark)
        .task {
            try? await Task.sleep(for: .milliseconds(200))
            model.startIfNeeded()
        }
    }

    private var topBar: some View {
        HStack(alignment: .center) {
            VStack(alignment: .leading, spacing: 2) {
                Text("2225 Dash")
                    .font(.system(size: 28, weight: .black, design: .rounded))
                Text("Pit-ready live telemetry")
                    .font(.system(size: 12, weight: .semibold, design: .rounded))
                    .foregroundStyle(.secondary)
            }

            Spacer()

            Button {
                ipFieldFocused = false
                model.refreshConnection()
            } label: {
                Image(systemName: "arrow.clockwise")
                    .font(.system(size: 15, weight: .bold))
                    .frame(width: 40, height: 40)
            }
            .buttonStyle(.plain)
            .background(Color.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        }
    }

    private var headerCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(alignment: .center) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Live Robot Telemetry")
                        .font(.system(size: 20, weight: .black, design: .rounded))
                    Text(model.connectionSummary)
                        .font(.system(size: 12, weight: .medium, design: .rounded))
                        .foregroundStyle(.secondary)
                }

                Spacer()

                HStack(spacing: 8) {
                    Circle()
                        .fill(model.isConnected ? Color.green : Color.orange)
                        .frame(width: 10, height: 10)
                    Text(model.isConnected ? "LIVE" : "OFFLINE")
                        .font(.system(size: 11, weight: .bold, design: .rounded))
                        .foregroundStyle(model.isConnected ? Color.green : Color.orange)
                }
                .padding(.horizontal, 9)
                .padding(.vertical, 7)
                .background(.white.opacity(0.06), in: Capsule())
            }

            HStack(spacing: 10) {
                TextField("Robot IP", text: $model.robotIP)
                    .focused($ipFieldFocused)
                    .textInputAutocapitalization(.never)
                    .autocorrectionDisabled()
                    .keyboardType(.numbersAndPunctuation)
                    .font(.system(size: 18, weight: .semibold, design: .rounded))
                    .padding(.horizontal, 14)
                    .frame(height: 50)
                    .background(Color.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 14, style: .continuous))

                Button {
                    ipFieldFocused = false
                    model.refreshConnection()
                } label: {
                    Text(model.isConnected ? "Reconnect" : "Connect")
                        .font(.system(size: 16, weight: .bold, design: .rounded))
                        .frame(width: 112, height: 50)
                }
                .buttonStyle(.borderedProminent)
                .tint(Color(red: 0.11, green: 0.56, blue: 0.97))
            }

            if let status = model.sotmStatus, !status.isEmpty, status != "IDLE" {
                Text("SOTM: \(status)")
                    .font(.system(size: 13, weight: .semibold, design: .rounded))
                    .foregroundStyle(.white.opacity(0.86))
                    .padding(.horizontal, 10)
                    .padding(.vertical, 7)
                    .background(Color.white.opacity(0.06), in: Capsule())
            }
        }
        .padding(16)
        .background(
            RoundedRectangle(cornerRadius: 20, style: .continuous)
                .fill(Color.white.opacity(0.07))
                .overlay(
                    RoundedRectangle(cornerRadius: 20, style: .continuous)
                        .stroke(Color.white.opacity(0.08), lineWidth: 1)
                )
        )
    }

    private var quickStats: some View {
        HStack(spacing: 12) {
            MiniStatCard(
                title: "Flywheel",
                value: model.displayRPM(model.flywheelRPM),
                subtitle: model.displayRPM(model.flywheelSetpoint),
                accent: Color.cyan
            )
            MiniStatCard(
                title: "Turret",
                value: model.displayAngle(model.turretAngleDeg),
                subtitle: model.turretCRTStatus ?? "CRT",
                accent: Color.orange
            )
            MiniStatCard(
                title: "Loader",
                value: model.displayDuty(model.loaderDuty),
                subtitle: model.loaderAutoReversing == true ? "Clearing" : "Ready",
                accent: Color.yellow
            )
        }
    }

    private var telemetryGrid: some View {
        LazyVStack(spacing: 12) {
            DashboardCard(title: "Flywheel", accent: .cyan) {
                DashboardRow(label: "Measured RPM", value: model.displayRPM(model.flywheelRPM))
                DashboardRow(label: "Raw REV RPM", value: model.displayRPM(model.flywheelRawRPM))
                DashboardRow(label: "Setpoint", value: model.displayRPM(model.flywheelSetpoint))
                DashboardRow(label: "Error", value: model.displayRPM(model.flywheelError))
                DashboardRow(label: "Duty", value: model.displayDuty(model.flywheelDuty))
                DashboardRow(label: "At Speed", value: model.booleanText(model.flywheelAtSpeed))
            }

            DashboardCard(title: "Turret", accent: .orange) {
                DashboardRow(label: "Angle", value: model.displayAngle(model.turretAngleDeg))
                DashboardRow(label: "Target", value: model.displayAngle(model.turretTargetDeg))
                DashboardRow(label: "CRT Status", value: model.turretCRTStatus ?? "---")
                DashboardRow(label: "CRT Seed", value: model.displayAngle(model.turretCRTSeedDeg))
                DashboardRow(label: "CRT Error", value: model.displayRotations(model.turretCRTErrorRot))
                DashboardRow(label: "Spread", value: model.displayCRTSpread(model.turretCRTSpread1, model.turretCRTSpread2))
            }

            DashboardCard(title: "Shoot On The Move", accent: .purple) {
                DashboardRow(label: "Status", value: model.sotmStatus ?? "---")
                DashboardRow(label: "Distance", value: model.displayMeters(model.sotmDistance))
                DashboardRow(label: "Flywheel", value: model.displayRPM(model.sotmFlywheelRPM))
                DashboardRow(label: "Turret", value: model.displayAngle(model.sotmTurretDeg))
                DashboardRow(label: "In Range", value: model.booleanText(model.sotmInRange))
                DashboardRow(label: "Active", value: model.booleanText(model.sotmActive))
            }

            DashboardCard(title: "Intake", accent: .green) {
                DashboardRow(label: "Pivot", value: model.displayAngle(model.intakeAngleDeg))
                DashboardRow(label: "Target", value: model.displayAngle(model.intakeTargetDeg))
                DashboardRow(label: "Error", value: model.displayAngle(model.intakeErrorDeg))
                DashboardRow(label: "Current", value: model.displayAmps(model.intakeCurrent))
                DashboardRow(label: "Roller", value: model.displayDuty(model.intakeRollerDuty))
            }

            DashboardCard(title: "Connection", accent: .blue) {
                DashboardRow(label: "Topics", value: "\(model.topicCount)")
                DashboardRow(label: "Cached Values", value: "\(model.valueCount)")
                DashboardRow(label: "Robot IP", value: model.robotIP)
                DashboardRow(label: "Socket", value: model.isConnected ? "Connected" : "Disconnected")
            }
        }
    }
}

private struct DashboardCard<Content: View>: View {
    let title: String
    let accent: Color
    @ViewBuilder let content: Content

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .fill(accent)
                    .frame(width: 10, height: 10)
                Text(title)
                    .font(.system(size: 16, weight: .heavy, design: .rounded))
                Spacer()
            }

            content
        }
        .padding(16)
        .background(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .fill(Color.white.opacity(0.06))
                .overlay(
                    RoundedRectangle(cornerRadius: 18, style: .continuous)
                        .stroke(Color.white.opacity(0.08), lineWidth: 1)
                )
        )
    }
}

private struct DashboardRow: View {
    let label: String
    let value: String

    var body: some View {
        HStack(alignment: .firstTextBaseline, spacing: 12) {
            Text(label.uppercased())
                .font(.system(size: 10, weight: .bold, design: .rounded))
                .kerning(1.0)
                .foregroundStyle(.secondary)
            Spacer(minLength: 12)
            Text(value)
                .font(.system(size: 18, weight: .bold, design: .rounded))
                .multilineTextAlignment(.trailing)
        }
    }
}

private struct MiniStatCard: View {
    let title: String
    let value: String
    let subtitle: String
    let accent: Color

    var body: some View {
        VStack(alignment: .leading, spacing: 7) {
            HStack {
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .fill(accent)
                    .frame(width: 8, height: 8)
                Text(title)
                    .font(.system(size: 12, weight: .bold, design: .rounded))
                    .foregroundStyle(.secondary)
            }
            Text(value)
                .font(.system(size: 21, weight: .black, design: .rounded))
                .minimumScaleFactor(0.65)
                .lineLimit(1)
            Text(subtitle)
                .font(.system(size: 12, weight: .semibold, design: .rounded))
                .foregroundStyle(.secondary)
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(13)
        .background(
            RoundedRectangle(cornerRadius: 16, style: .continuous)
                .fill(Color.white.opacity(0.06))
                .overlay(
                    RoundedRectangle(cornerRadius: 16, style: .continuous)
                        .stroke(Color.white.opacity(0.08), lineWidth: 1)
                )
        )
    }
}

#Preview {
    ContentView()
}
