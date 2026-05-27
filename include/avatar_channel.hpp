#pragma once

// Thin wrapper around UdpReliable for the avatar command channel.

#include "network/udp_reliable.hpp"
#include "network/common.hpp"
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>

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

    void requestState(SysState state);
    void setLocalState(SysState state);

    bool      isAlive()        const;
    void      resetAliveTimer();
    SysState  getRemoteState() const { return remote_state_.load(); }

private:
    void heartbeatLoop();

    UdpReliable             channel_;
    std::atomic<SysState>   remote_state_{SysState::OFFLINE};
    std::atomic<bool>       running_{false};
    std::thread             heartbeat_thread_;
    std::chrono::steady_clock::time_point start_time_;
};
