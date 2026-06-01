#include "keyboard_haptic_device.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>

#ifdef _WIN32
#  include <conio.h>
#else
#  include <termios.h>
#  include <unistd.h>
#  include <sys/select.h>
#endif

static constexpr double TRANS_STEP = 0.002;   // 2 mm per keypress
static constexpr double ROT_STEP   = 0.05236; // ~3 deg per keypress

KeyboardHapticDevice::KeyboardHapticDevice(int id)
    : name_("keyboard_" + std::to_string(id))
{}

KeyboardHapticDevice::~KeyboardHapticDevice() {
    close();
}

bool KeyboardHapticDevice::open() {
    // Spawn input thread, print key map
    prev_read_time_   = std::chrono::high_resolution_clock::now();
    last_force_print_ = prev_read_time_;
    connected_        = true;
    running_          = true;
    thread_ = std::thread(&KeyboardHapticDevice::inputThread, this);
    std::cout << "[KeyboardHapticDevice] Opened '" << name_ << "'\n"
              << "  W/S = +Y/-Y    A/D = -X/+X    R/F = +Z/-Z\n"
              << "  I/K = pitch    J/L = yaw       U/O = roll\n"
              << "  G = gripper    T = reset pose\n"
              << "  SPACE = engage   ESC/Q = idle\n";
    return true;
}

void KeyboardHapticDevice::close() {
    // Stop input thread and restore terminal
    if (!running_.exchange(false)) return;
    connected_ = false;
    if (thread_.joinable()) thread_.join();
}

void KeyboardHapticDevice::zero() {
    // No force actuator; zero is a no-op — pose preserved for origin capture
}

bool KeyboardHapticDevice::isConnected() const {
    return connected_.load();
}

const char* KeyboardHapticDevice::name() const {
    return name_.c_str();
}

int KeyboardHapticDevice::consumeSessionKey() {
    // Atomically read and clear the pending session key
    return session_key_.exchange(0);
}

void KeyboardHapticDevice::enableForce(bool enable) {
    // Track force-enable flag; used to gate force display
    force_enabled_ = enable;
}

void KeyboardHapticDevice::setForce(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) {
    // Display received haptic wrench throttled at ~10 Hz
    if (!force_enabled_) return;
    auto now = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration<double>(now - last_force_print_).count() < 0.1) return;
    last_force_print_ = now;
    std::cout << "\r[haptic] F=["
              << std::fixed << std::setprecision(2)
              << force.x()  << " " << force.y()  << " " << force.z()  << "] "
              << "T=[" << torque.x() << " " << torque.y() << " " << torque.z() << "]   "
              << std::flush;
}

HapticState KeyboardHapticDevice::readState() {
    // Snapshot pose and compute finite-difference velocity
    HapticState s{};
    auto now = std::chrono::high_resolution_clock::now();
    double dt = std::max(std::chrono::duration<double>(now - prev_read_time_).count(), 1e-4);

    Eigen::Vector3d    pos;
    Eigen::Quaterniond ori;
    {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        pos = position_;
        ori = orientation_;
    }

    s.position    = pos;
    s.orientation = ori;
    s.twist.head<3>() = (pos - prev_position_) / dt;

    Eigen::Quaterniond dq = prev_orientation_.conjugate() * ori;
    if (dq.w() < 0.0) dq.coeffs() *= -1.0;
    s.twist.tail<3>() = 2.0 * dq.vec() / dt;

    s.gripper_closed = gripper_closed_.load();
    s.gripper_angle  = s.gripper_closed ? 0.0 : 30.0;
    s.is_valid       = connected_.load();

    prev_position_    = pos;
    prev_orientation_ = ori;
    prev_read_time_   = now;

    return s;
}

void KeyboardHapticDevice::processKey(int key) {
    // Map keypress to incremental pose delta and session keys
    std::lock_guard<std::mutex> lk(pose_mtx_);

    switch (key) {
        // Translation
        case 'w': case 'W': position_.y() +=  TRANS_STEP; break;
        case 's': case 'S': position_.y() -= TRANS_STEP;  break;
        case 'a': case 'A': position_.x() -= TRANS_STEP;  break;
        case 'd': case 'D': position_.x() +=  TRANS_STEP; break;
        case 'r': case 'R': position_.z() +=  TRANS_STEP; break;
        case 'f': case 'F': position_.z() -= TRANS_STEP;  break;

        // Rotation — local frame increments applied on the right
        case 'i': case 'I':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd( ROT_STEP, Eigen::Vector3d::UnitX()))).normalized(); break;
        case 'k': case 'K':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd(-ROT_STEP, Eigen::Vector3d::UnitX()))).normalized(); break;
        case 'j': case 'J':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd( ROT_STEP, Eigen::Vector3d::UnitZ()))).normalized(); break;
        case 'l': case 'L':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd(-ROT_STEP, Eigen::Vector3d::UnitZ()))).normalized(); break;
        case 'u': case 'U':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd( ROT_STEP, Eigen::Vector3d::UnitY()))).normalized(); break;
        case 'o': case 'O':
            orientation_ = (orientation_ * Eigen::Quaterniond(Eigen::AngleAxisd(-ROT_STEP, Eigen::Vector3d::UnitY()))).normalized(); break;

        // Gripper toggle
        case 'g': case 'G':
            gripper_closed_ = !gripper_closed_.load(); break;

        // Full pose reset
        case 't': case 'T':
            position_    = Eigen::Vector3d::Zero();
            orientation_ = Eigen::Quaterniond::Identity();
            break;

        // Session keys forwarded to TeleopSession
        case ' ': case 27: case 'q': case 'Q':
            session_key_ = key;
            break;

        default: break;
    }

    if (orientation_.w() < 0.0) orientation_.coeffs() *= -1.0;
}

void KeyboardHapticDevice::inputThread() {
    // Blocking stdin reader; owns terminal raw mode for the session lifetime
#ifdef _WIN32
    while (running_) {
        if (_kbhit()) {
            int key = _getch();
            // skip extended key scan-code prefix (0x00 or 0xE0)
            if (key == 0 || key == 0xE0) { _getch(); }
            else                         { processKey(key); }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#else
    termios oldt{};
    tcgetattr(STDIN_FILENO, &oldt);
    termios newt   = oldt;
    newt.c_lflag  &= ~static_cast<tcflag_t>(ICANON | ECHO);
    newt.c_cc[VMIN]  = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{0, 50000};   // 50 ms select timeout → responsive shutdown
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
            int key = getchar();
            if (key != EOF) processKey(key);
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
#endif
}
