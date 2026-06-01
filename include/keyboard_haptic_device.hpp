#pragma once

#include "haptic_device.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>

class KeyboardHapticDevice : public IHapticDevice {
public:
    explicit KeyboardHapticDevice(int id);
    ~KeyboardHapticDevice();

    bool        open()                                                     override;
    void        close()                                                    override;
    void        zero()                                                     override;
    bool        isConnected()                                        const override;
    HapticState readState()                                                override;
    void        setForce(const Eigen::Vector3d& f, const Eigen::Vector3d& t) override;
    void        enableForce(bool enable)                                   override;
    const char* name()                                               const override;
    int         consumeSessionKey()                                        override;

private:
    void inputThread();
    void processKey(int key);

    std::string       name_;
    std::atomic<bool> running_       {false};
    std::atomic<bool> connected_     {false};
    std::atomic<bool> force_enabled_ {false};
    std::atomic<int>  session_key_   {0};
    std::atomic<bool> gripper_closed_{false};

    std::mutex         pose_mtx_;
    Eigen::Vector3d    position_    {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_ {Eigen::Quaterniond::Identity()};

    Eigen::Vector3d    prev_position_    {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond prev_orientation_ {Eigen::Quaterniond::Identity()};
    std::chrono::high_resolution_clock::time_point prev_read_time_;
    std::chrono::high_resolution_clock::time_point last_force_print_;

    std::thread thread_;
};
