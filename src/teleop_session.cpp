#include "teleop_session.hpp"
#include "udp_arm_channel.hpp"
#include <iostream>
#include <chrono>
#include <filesystem>
#ifdef _WIN32
#  include <conio.h>
#else
#  include <termios.h>
#  include <unistd.h>
#  include <fcntl.h>
#endif

#ifdef WITH_SIGMA
#  include "sigma_device.hpp"
#else
#  include "haptic_device.hpp"
#  include "mock_haptic_device.hpp"
#  include "keyboard_haptic_device.hpp"
#endif

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

// Non-blocking single keypress check; returns 0 if no key is waiting.
static int pollKey() {
#ifdef _WIN32
    return _kbhit() ? _getch() : 0;
#else
    termios oldt{};
    tcgetattr(STDIN_FILENO, &oldt);
    termios newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return (ch == EOF) ? 0 : ch;
#endif
}

TeleopSession::TeleopSession(const std::string& system_config_path) {
    YAML::Node sys = YAML::LoadFile(system_config_path);

    log_dir_ = sys["session"]["log_dir"].as<std::string>("log");
    std::filesystem::create_directories(log_dir_);
    bool mock_motion = sys["session"]["mock_motion"].as<bool>(false);

    AvatarChannelConfig av_cfg;
    av_cfg.remote_ip    = sys["avatar"]["remote_ip"].as<std::string>();
    av_cfg.send_port    = sys["avatar"]["send_port"].as<int>();
    av_cfg.receive_port = sys["avatar"]["receive_port"].as<int>();
    av_cfg.frequency_hz = sys["avatar"]["frequency"].as<int>();
    avatar_channel_ = std::make_unique<AvatarChannel>(av_cfg);

    std::cout << "Loading device with " << av_cfg.remote_ip << std::endl;

    for (const auto& dev : sys["devices"]) {
        std::string side       = dev["side"].as<std::string>();
        std::string haptic_cfg = dev["haptic_config"].as<std::string>();

        YAML::Node hcfg = YAML::LoadFile(haptic_cfg);
        int device_id   = hcfg["device_id"].as<int>();

        ArmChannelConfig arm_cfg;
        arm_cfg.remote_ip    = dev["arm"]["remote_ip"].as<std::string>();
        arm_cfg.send_port    = dev["arm"]["send_port"].as<int>();
        arm_cfg.receive_port = dev["arm"]["receive_port"].as<int>();
        arm_cfg.frequency_hz = dev["arm"]["frequency"].as<int>();
        arm_cfg.side_name    = side;
        auto arm = std::make_unique<UdpArmChannel>(arm_cfg);

        std::unique_ptr<IHapticDevice> haptic;
#ifdef WITH_SIGMA
        haptic = std::make_unique<SigmaDevice>(device_id);
#else
        if (mock_motion) {
            std::string profile_str = sys["session"]["mock_profile"].as<std::string>("free_x");
            if (profile_str == "keyboard") {
                haptic = std::make_unique<KeyboardHapticDevice>(device_id);
                keyboard_mode_ = true;
                std::cout << "[TeleopSession] Using KeyboardHapticDevice for " << side << "\n";
            } else {
                MockProfile profile = MockProfile::FREE_X;
                if      (profile_str == "contact_z")      profile = MockProfile::CONTACT_Z;
                else if (profile_str == "contact_z_down") profile = MockProfile::CONTACT_Z_DOWN;
                haptic = std::make_unique<MockHapticDevice>(device_id, profile);
                std::cout << "[TeleopSession] Using MockHapticDevice for " << side << " (profile: " << profile_str << ")\n";
            }
        } else {
            haptic = std::make_unique<NullHapticDevice>(device_id);
            std::cout << "[TeleopSession] Using NullHapticDevice for " << side << "\n";
        }
#endif

        controllers_.push_back(std::make_unique<TeleopController>(
            std::move(haptic), std::move(arm), hcfg, log_dir_));
    }
}

TeleopSession::~TeleopSession() {
    stop();
}

void TeleopSession::run() {
    avatar_channel_->start();
    for (auto& ctrl : controllers_) ctrl->start();

    running_ = true;
    state_   = SysState::IDLE;
    avatar_channel_->setLocalState(state_);

    std::cout << "[TeleopSession] Running.\n"
              << "  SPACE  - home then engage\n"
              << "  ESC    - disengage\n"
              << "  Ctrl-C - quit\n";

    constexpr auto PERIOD = std::chrono::milliseconds(10);
    auto next = std::chrono::steady_clock::now();

    while (running_) {
        updateStateMachine();
        next += PERIOD;
        std::this_thread::sleep_until(next);
    }

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

void TeleopSession::updateStateMachine() {
    int key = 0;
    if (!keyboard_mode_) {
        key = pollKey();
    } else {
        for (auto& ctrl : controllers_) {
            int k = ctrl->pollSessionKey();
            if (k != 0) { key = k; break; }
        }
    }
    SysState remote = avatar_channel_->getRemoteState();

    bool avatar_stopped = (remote == SysState::FAULT || remote == SysState::STOP) ||
                          (remote == SysState::OFFLINE && !avatar_channel_->isAlive());

    if (key == 27 || key == 'q') {
        // ESC / q always returns to IDLE regardless of current state
        requestAllDevices(SysState::IDLE);
        state_ = SysState::IDLE;
        avatar_channel_->setLocalState(state_);
        avatar_channel_->requestState(SysState::IDLE);
        std::cout << "[TeleopSession] IDLE\n";
        return;
    }

    if (state_ == SysState::IDLE) {
        if (key == ' ') {
            state_ = SysState::HOMING;
            avatar_channel_->setLocalState(state_);
            avatar_channel_->requestState(SysState::HOMING);
            std::cout << "[TeleopSession] HOMING requested — waiting for avatar...\n";
        }
    } else if (state_ == SysState::HOMING) {
        if (remote == SysState::AWAITING) {
            // Arms have finished homing; capture origins and engage
            for (auto& ctrl : controllers_) ctrl->captureOrigin();
            requestAllDevices(SysState::ENGAGED);
            state_ = SysState::ENGAGED;
            avatar_channel_->setLocalState(state_);
            avatar_channel_->requestState(SysState::ENGAGED);
            std::cout << "[TeleopSession] ENGAGED\n";
        }
    } else if (state_ == SysState::ENGAGED) {
        if (avatar_stopped) {
            requestAllDevices(SysState::IDLE);
            state_ = SysState::IDLE;
            avatar_channel_->setLocalState(state_);
            avatar_channel_->requestState(SysState::IDLE);
            std::cout << "[TeleopSession] IDLE (avatar triggered)\n";
        }
    }
}

void TeleopSession::requestAllDevices(SysState state) {
    for (auto& ctrl : controllers_) {
        if (state == SysState::ENGAGED)
            ctrl->setEngaged(true);
        else
            ctrl->setEngaged(false);
    }
}
