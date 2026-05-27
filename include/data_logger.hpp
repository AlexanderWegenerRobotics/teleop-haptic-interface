#pragma once

// ─── data_logger.hpp ──────────────────────────────────────────────────────────
// Async templated CSV logger.
// Usage pattern (same as avatar project):
//
//   struct MyEntry { double time; float x, y, z; };
//   auto headerFn = []() { return "time,x,y,z\n"; };
//   auto rowFn    = [](const MyEntry& e) {
//       return std::to_string(e.time) + "," + std::to_string(e.x) + "\n"; };
//
//   DataLogger<MyEntry> log("path/to/file.csv", headerFn, rowFn);
//   log.start();
//   log.enable(true);
//   log.write(entry);   // non-blocking
//   log.stop();
// ─────────────────────────────────────────────────────────────────────────────

#include <string>
#include <fstream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include <iostream>

template<typename Entry>
class DataLogger {
public:
    using HeaderFn = std::function<std::string()>;
    using RowFn    = std::function<std::string(const Entry&)>;

    DataLogger(std::string path, HeaderFn header_fn, RowFn row_fn)
        : path_(std::move(path))
        , header_fn_(std::move(header_fn))
        , row_fn_(std::move(row_fn))
    {}

    ~DataLogger() { stop(); }

    DataLogger(const DataLogger&)            = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    // Open the file and start the writer thread.
    void start() {
        if (running_) return;
        openFile(path_);
        running_ = true;
        thread_  = std::thread(&DataLogger::run, this);
    }

    // Drain the queue, close the file, and join the thread.
    void stop() {
        if (!running_) return;
        running_ = false;
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        if (file_.is_open()) file_.close();
    }

    // Enable or disable writing (start() must be called first).
    void enable(bool on) { enabled_ = on; }

    // Queue an entry for writing (non-blocking; called from control thread).
    void write(const Entry& entry) {
        if (!enabled_ || !running_) return;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(entry);
        }
        cv_.notify_one();
    }

    // Close the current file and re-open at a new path.
    // Intended for episode-based logging (new file per episode).
    void restart(const std::string& new_path) {
        // Drain remaining entries to the old file first
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this] { return queue_.empty(); });
        }
        if (file_.is_open()) file_.close();
        path_ = new_path;
        openFile(path_);
    }

private:
    void openFile(const std::string& path) {
        file_.open(path, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            std::cerr << "[DataLogger] Failed to open: " << path << "\n";
            return;
        }
        file_ << header_fn_();
        file_.flush();
    }

    void run() {
        while (running_ || !queueEmpty()) {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

            while (!queue_.empty()) {
                Entry entry = queue_.front();
                queue_.pop();
                lock.unlock();

                if (file_.is_open()) {
                    file_ << row_fn_(entry);
                }
                lock.lock();
            }

            if (file_.is_open()) file_.flush();
        }
    }

    bool queueEmpty() {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.empty();
    }

    std::string   path_;
    HeaderFn      header_fn_;
    RowFn         row_fn_;

    std::ofstream            file_;
    std::queue<Entry>        queue_;
    std::mutex               mtx_;
    std::condition_variable  cv_;
    std::thread              thread_;
    std::atomic<bool>        running_{false};
    std::atomic<bool>        enabled_{false};
};
