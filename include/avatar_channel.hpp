#pragma once

// ─── avatar_channel.hpp ───────────────────────────────────────────────────────
// Thin wrapper around UdpReliable for the avatar command channel.
// Handles state-change commands and heartbeat/device-event reception.
// ─────────────────────────────────────────────────────────────────────────────

#include "network/udp_reliable.hpp"
#include "network/common.hpp"
#include <string>
#include <functional>
#include <atomic>

struct AvatarChannelConfig {
    std::string remote_ip;
    int         send_port;      // avatar listens here (we send commands TO it)
    int         receive_port;   // we listen here (avatar sends heartbeat TO us)
    int         frequency_hz;
};

class AvatarChannel {
public:
    explicit AvatarChannel(const AvatarChannelConfig& cfg);

    void start();
    void stop();

    // Send a state-change request to the avatar (HOMING, ENGAGED, IDLE …)
    void requestState(SysState state);

    // Inform the avatar of our current state (stamped on heartbeat replies)
    void setLocalState(SysState state);

    bool      isAlive()        const;
    void      resetAliveTimer();
    SysState  getRemoteState() const { return remote_state_.load(); }

private:
    UdpReliable             channel_;
    std::atomic<SysState>   remote_state_{SysState::OFFLINE};
};
