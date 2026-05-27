#include "avatar_channel.hpp"
#include <iostream>
#include <map>
#include <cstdint>

AvatarChannel::AvatarChannel(const AvatarChannelConfig& cfg)
    : channel_([&] {
        UdpReliableConfig rc;
        rc.transport.bind_address  = "0.0.0.0";
        rc.transport.bind_port     = cfg.receive_port;
        rc.transport.remote_ip     = cfg.remote_ip;
        rc.transport.remote_port   = cfg.send_port;
        rc.transport.recv_timeout_us = 100;
        rc.poll_rate_hz            = cfg.frequency_hz;
        return rc;
    }())
{
    // Handle heartbeat — update remote state from envelope
    channel_.registerHandler("heartbeat",
        [this](const ReliableEnvelope& env, const msgpack::object& payload) {
            (void)payload;
            remote_state_.store(static_cast<SysState>(env.state));
        });

    // Handle device events (recovery complete, etc.)
    channel_.registerHandler("device_event",
        [](const ReliableEnvelope& env, const msgpack::object& payload) {
            (void)env;
            try {
                std::map<std::string, msgpack::object> fields;
                payload.convert(fields);
                std::string device = fields.at("device").as<std::string>();
                std::string event  = fields.at("event").as<std::string>();
                std::cout << "[AvatarChannel] device_event: "
                          << device << " → " << event << "\n";
            } catch (...) {}
        });
}

void AvatarChannel::start() {
    start_time_ = std::chrono::steady_clock::now();
    channel_.resetAliveTimer();
    channel_.start();
    running_ = true;
    heartbeat_thread_ = std::thread(&AvatarChannel::heartbeatLoop, this);
}

void AvatarChannel::stop() {
    running_ = false;
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    channel_.stop();
}

void AvatarChannel::heartbeatLoop() {
    // Send heartbeat every 500 ms so the avatar's alive-timer doesn't expire.
    while (running_) {
        int64_t uptime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
        msgpack::sbuffer buf;
        msgpack::pack(buf, std::map<std::string, int64_t>{{"uptime_ms", uptime_ms}});
        channel_.send("heartbeat", buf, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void AvatarChannel::requestState(SysState state) {
    msgpack::sbuffer buf;
    msgpack::pack(buf, std::map<std::string, uint8_t>{
        {"requested_state", static_cast<uint8_t>(state)}
    });
    channel_.send("state_change", buf, /*ack=*/true);
    std::cout << "[AvatarChannel] Requesting state: "
              << sysStateStr(state) << "\n";
}

void AvatarChannel::setLocalState(SysState state) {
    channel_.setState(state);
}

bool AvatarChannel::isAlive() const {
    return channel_.isAlive();
}

void AvatarChannel::resetAliveTimer() {
    channel_.resetAliveTimer();
}
