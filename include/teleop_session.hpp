#pragma once

// ─── teleop_session.hpp ───────────────────────────────────────────────────────
// Top-level orchestrator: owns both TeleopControllers, the AvatarChannel,
// and the system-level state machine.
// ─────────────────────────────────────────────────────────────────────────────

#include "teleop_controller.hpp"
#include "avatar_channel.hpp"
#include "network/common.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <atomic>
#include <thread>
#include <vector>

class TeleopSession {
public:
    explicit TeleopSession(const std::string& system_config_path);
    ~TeleopSession();

    // Blocking main loop.  Returns when stop() is called.
    void run();
    void stop();

private:
    void updateStateMachine();
    void requestAllDevices(SysState state);

    // Populated by loadConfig()
    std::unique_ptr<AvatarChannel>                   avatar_channel_;
    std::vector<std::unique_ptr<TeleopController>>   controllers_;

    SysState              state_{SysState::IDLE};
    std::atomic<bool>     running_{false};
    std::string           log_dir_;
    bool                  keyboard_mode_{false};
};
