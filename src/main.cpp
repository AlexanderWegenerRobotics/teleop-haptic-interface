#include "teleop_session.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_stop{false};
static TeleopSession*    g_session = nullptr;

static void signalHandler(int /*sig*/) {
    g_stop = true;
    if (g_session) g_session->stop();
}

int main(int argc, char* argv[]) {
    std::string config_path = "config/system.yaml";
    if (argc > 1) config_path = argv[1];

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "[main] Loading config: " << config_path << "\n";

    try {
        TeleopSession session(config_path);
        g_session = &session;
        session.run();
    } catch (const std::exception& e) {
        std::cerr << "[main] Fatal error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "[main] Clean exit.\n";
    return 0;
}
