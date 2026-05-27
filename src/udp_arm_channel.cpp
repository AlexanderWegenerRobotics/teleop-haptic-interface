#include "udp_arm_channel.hpp"
#include <cstring>

UdpArmChannel::UdpArmChannel(const ArmChannelConfig& cfg)
    : name_("arm_" + cfg.side_name)
    , device_id_(cfg.side_name == "right" ? DeviceId::RIGHT_ARM : DeviceId::LEFT_ARM)
    , stream_([&] {
        UdpStreamConfig sc;
        sc.transport.bind_address  = "0.0.0.0";
        sc.transport.bind_port     = cfg.receive_port;  // we listen for arm state here
        sc.transport.remote_ip     = cfg.remote_ip;
        sc.transport.remote_port   = cfg.send_port;     // arm listens for commands here
        sc.transport.recv_timeout_us = 100;
        sc.send_rate_hz            = cfg.frequency_hz;
        return sc;
    }())
{}

void UdpArmChannel::start() {
    stream_.start();
}

void UdpArmChannel::stop() {
    stream_.stop();
}

void UdpArmChannel::sendCommand(const ArmCommand& cmd) {
    ArmCommandMsg msg{};
    msg.header.timestamp_ns = timestamp_ns();
    msg.header.device_id    = device_id_;
    msg.position[0]   = static_cast<float>(cmd.position.x());
    msg.position[1]   = static_cast<float>(cmd.position.y());
    msg.position[2]   = static_cast<float>(cmd.position.z());
    // Quaternion stored as [w, x, y, z] — must match avatar convention
    msg.quaternion[0] = static_cast<float>(cmd.orientation.w());
    msg.quaternion[1] = static_cast<float>(cmd.orientation.x());
    msg.quaternion[2] = static_cast<float>(cmd.orientation.y());
    msg.quaternion[3] = static_cast<float>(cmd.orientation.z());
    msg.gripper       = cmd.gripper_closed ? 1.0f : 0.0f;
    stream_.setSendData(msg);
}

ArmState UdpArmChannel::getState() {
    ArmStateMsg msg = stream_.getRecvData();
    last_state_.position    = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
    // Quaternion from avatar: [w, x, y, z]
    last_state_.orientation = Eigen::Quaterniond(msg.quaternion[0], msg.quaternion[1], msg.quaternion[2], msg.quaternion[3]);
    last_state_.orientation.normalize();
    last_state_.is_recovering = (msg.recovering != 0);
    last_state_.is_valid      = isAlive();
    last_state_.timestamp     = std::chrono::steady_clock::now();
    return last_state_;
}

bool UdpArmChannel::hasNewState() const {
    return stream_.hasNew();
}

bool UdpArmChannel::isAlive() const {
    return stream_.isAlive();
}
