#include "avatar_channel.hpp"
#include <iostream>
#include <map>

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
    // Handle heartbeat from avatar
    channel_.registerHandler("heartbeat",
        [](const ReliableEnvelope& env, const msgpack::object& payload) {
            (void)env; (void)payload;
            // Could log uptime here if needed
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
    channel_.resetAliveTimer();
    channel_.start();
}

void AvatarChannel::stop() {
    channel_.stop();
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
