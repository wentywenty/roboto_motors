/**
 * @file
 * This file declares an interface to SocketCANFD,
 * to facilitates frame transmission and reception.
 */

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <atomic>
#include <boost/lockfree/queue.hpp>
#include <condition_variable>
#include <cstdbool>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

constexpr const int FD_INIT_FD = -1;
constexpr const int FD_TIMEOUT_SEC = 0;
constexpr const int FD_TIMEOUT_USEC = 1000;
constexpr const int FD_TX_QUEUE_SIZE = 4096;
constexpr const int FD_MAX_RETRY_COUNT = 3;

using LFQueueFD = boost::lockfree::queue<canfd_frame, boost::lockfree::fixed_sized<true>>;
using CanFdCbkFunc = std::function<void(const canfd_frame &)>;
using CanFdCbkId = uint16_t;
using CanFdCbkMap = std::unordered_map<CanFdCbkId, CanFdCbkFunc>;
using CanFdCbkKeyExtractor = std::function<CanFdCbkId(const canfd_frame &)>;

class MotorsSocketCANFD {
   private:
    std::string interface_;  // The network interface name
    int sockfd_ = -1;        // The file descriptor for the CAN socket
    std::atomic<bool> receiving_;
    LFQueueFD tx_queue_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;

    sockaddr_can addr_;      // The address of the CAN socket
    ifreq if_request_;       // The network interface request

    /// Receiving
    std::thread receiver_thread_;
    CanFdCbkMap canfd_callback_list_;
    std::mutex canfd_callback_mutex_;
    CanFdCbkKeyExtractor key_extractor_ = [](const canfd_frame &frame) -> CanFdCbkId {
        return static_cast<CanFdCbkId>(frame.can_id);
    };

    /// Transmitting
    std::thread sender_thread_;
    std::atomic<int> send_sleep_us_{0};

    MotorsSocketCANFD(std::string interface);

    static std::shared_ptr<MotorsSocketCANFD> createInstance(const std::string &interface) {
        return std::shared_ptr<MotorsSocketCANFD>(new MotorsSocketCANFD(interface));
    }
    static std::shared_ptr<spdlog::logger> logger_;
    static std::unordered_map<std::string, std::shared_ptr<MotorsSocketCANFD>> instances_;

   public:
    MotorsSocketCANFD(const MotorsSocketCANFD &) = delete;
    MotorsSocketCANFD &operator=(const MotorsSocketCANFD &) = delete;
    ~MotorsSocketCANFD();
    static void init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
    static std::shared_ptr<MotorsSocketCANFD> get(std::string interface) {
        if (!logger_) {
            logger_ = spdlog::get("motors");
            if (!logger_) {
                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
                logger_ = std::make_shared<spdlog::logger>("motors", std::begin(sinks), std::end(sinks));
                spdlog::register_logger(logger_);
            }
        }
        if (instances_.find(interface) == instances_.end()) instances_[interface] = createInstance(interface);
        return instances_[interface];
    }
    void open(std::string interface);
    void close();
    void transmit(const canfd_frame &frame);
    void add_canfd_callback(const CanFdCbkFunc callback, const CanFdCbkId id);
    void remove_canfd_callback(const CanFdCbkId id);
    void clear_canfd_callbacks();
    void set_canfd_key_extractor(CanFdCbkKeyExtractor extractor);
    void set_send_sleep(int us) { send_sleep_us_ = us; }
};
