#pragma once

#include "arm_channel.hpp"
#include "network/common.hpp"
#include "network/udp_stream.hpp"
#include <string>

// UdpArmChannel wraps a UdpStream<ArmStateMsg, ArmCommandMsg>
// and exposes the IArmChannel interface.

struct ArmChannelConfig {
    std::string remote_ip;
    int         send_port;      // arm listens here (we send commands TO it)
    int         receive_port;   // we listen here   (arm sends state TO us)
    int         frequency_hz;
    std::string side_name;      // "left" or "right"
};

class UdpArmChannel : public IArmChannel {
public:
    explicit UdpArmChannel(const ArmChannelConfig& cfg);
    ~UdpArmChannel() override { stop(); }

    void     start()                           override;
    void     stop()                            override;
    void     sendCommand(const ArmCommand& cmd) override;
    ArmState getState()                        override;
    bool     hasNewState()   const             override;
    bool     isAlive()       const             override;
    const char* name()       const             override { return name_.c_str(); }

private:
    using Stream = UdpStream<ArmStateMsg, ArmCommandMsg>;

    std::string       name_;
    DeviceId          device_id_;
    Stream            stream_;
    ArmState          last_state_;
};
