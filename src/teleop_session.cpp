#include "teleop_session.hpp"
#include "udp_arm_channel.hpp"
#include <iostream>
#include <chrono>
#include <filesystem>

// Conditionally include real or mock haptic device
#ifdef WITH_SIGMA
#  include "sigma_device.hpp"
#else
// TODO: include mock_haptic_device.hpp once implemented
#  include "haptic_device.hpp"  // interface only — placeholder
#endif

// ─── Placeholder mock (compiles without sigma SDK) ────────────────────────────
// Will be replaced by a proper MockHapticDevice in a later step.
#ifndef WITH_SIGMA
class NullHapticDevice : public IHapticDevice {
public:
    explicit NullHapticDevice(int /*id*/) {}
    bool open()  override { return true; }
    void close() override {}
    HapticState readState() override {
        HapticState s{};
        s.orientation = Eigen::Quaterniond::Identity();
        s.is_valid    = true;
        return s;
    }
    void setForce(const Eigen::Vector3d&, const Eigen::Vector3d&) override {}
    void enableForce(bool) override {}
    void zero()       override {}
    bool isConnected() const override { return true; }
    const char* name() const override { return "null_haptic"; }
};
#endif

// ─── Construction ─────────────────────────────────────────────────────────────

TeleopSession::TeleopSession(const std::string& system_config_path) {
    YAML::Node sys = YAML::LoadFile(system_config_path);

    log_dir_ = sys["session"]["log_dir"].as<std::string>("log");
    std::filesystem::create_directories(log_dir_);

    // ── Avatar command channel ────────────────────────────────────────────────
    AvatarChannelConfig av_cfg;
    av_cfg.remote_ip    = sys["avatar"]["remote_ip"].as<std::string>();
    av_cfg.send_port    = sys["avatar"]["send_port"].as<int>();
    av_cfg.receive_port = sys["avatar"]["receive_port"].as<int>();
    av_cfg.frequency_hz = sys["avatar"]["frequency"].as<int>();
    avatar_channel_ = std::make_unique<AvatarChannel>(av_cfg);

    // ── Per-device controllers ────────────────────────────────────────────────
    for (const auto& dev : sys["devices"]) {
        std::string side       = dev["side"].as<std::string>();
        std::string haptic_cfg = dev["haptic_config"].as<std::string>();

        YAML::Node hcfg = YAML::LoadFile(haptic_cfg);
        int device_id   = hcfg["device_id"].as<int>();

        // Arm channel
        ArmChannelConfig arm_cfg;
        arm_cfg.remote_ip    = dev["arm"]["remote_ip"].as<std::string>();
        arm_cfg.send_port    = dev["arm"]["send_port"].as<int>();
        arm_cfg.receive_port = dev["arm"]["receive_port"].as<int>();
        arm_cfg.frequency_hz = dev["arm"]["frequency"].as<int>();
        arm_cfg.side_name    = side;
        auto arm = std::make_unique<UdpArmChannel>(arm_cfg);

        // Haptic device
        std::unique_ptr<IHapticDevice> haptic;
#ifdef WITH_SIGMA
        haptic = std::make_unique<SigmaDevice>(device_id);
#else
        haptic = std::make_unique<NullHapticDevice>(device_id);
        std::cout << "[TeleopSession] No Sigma SDK — using NullHapticDevice for "
                  << side << "\n";
#endif

        controllers_.push_back(std::make_unique<TeleopController>(
            haptic.release(), arm.release(), hcfg, log_dir_));
    }
}

TeleopSession::~TeleopSession() {
    stop();
}

// ─── Main loop ────────────────────────────────────────────────────────────────

void TeleopSession::run() {
    avatar_channel_->start();
    for (auto& ctrl : controllers_) ctrl->start();

    running_ = true;
    state_   = SysState::IDLE;
    avatar_channel_->setLocalState(state_);

    std::cout << "[TeleopSession] Running. Press Ctrl-C to stop.\n";

    constexpr auto PERIOD = std::chrono::milliseconds(10);
    auto next = std::chrono::steady_clock::now();

    while (running_) {
        updateStateMachine();
        next += PERIOD;
        std::this_thread::sleep_until(next);
    }

    // Clean shutdown
    for (auto& ctrl : controllers_) {
        ctrl->setEngaged(false);
        ctrl->stop();
    }
    avatar_channel_->requestState(SysState::IDLE);
    avatar_channel_->stop();
}

void TeleopSession::stop() {
    running_ = false;
}

// ─── State machine ────────────────────────────────────────────────────────────
// Simplified: IDLE → HOMING → AWAITING → ENGAGED (→ PAUSED → AWAITING)
// The avatar drives transitions; we mirror them and propagate to controllers.

void TeleopSession::updateStateMachine() {
    // Stub: full state machine logic implemented in next step.
    // For now the session stays in IDLE until wired up to keyboard / GUI.
    (void)state_;
}

void TeleopSession::requestAllDevices(SysState state) {
    for (auto& ctrl : controllers_) {
        if (state == SysState::ENGAGED)
            ctrl->setEngaged(true);
        else
            ctrl->setEngaged(false);
    }
}
