#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include "udp_listener.hpp"

namespace mirena
{

    class LidarSocketPair
    {
    public:
        LidarSocketPair(){
            msop_socket_ = nullptr;
            difop_socket_ = nullptr;
        }
        /**
         * Constructor - acquires sockets automatically.
         *
         * @param self_ip Local IP address
         * @param msop_port MSOP port number
         * @param difop_port DIFOP port number
         * @param lidar_ip LIDAR IP address
         * @throws std::runtime_error if socket acquisition fails
         */
        LidarSocketPair(const in_addr_t &self_ip,
                        const in_addr_t &lidar_ip,
                        uint16_t msop_port,
                        uint16_t difop_port)
        {
            msop_socket_ = std::make_unique<UDPListener>(self_ip, msop_port, true, lidar_ip);
            difop_socket_ = std::make_unique<UDPListener>(self_ip, difop_port, true, lidar_ip);

            if ((msop_socket_->get_socket_fd() < 0) || (difop_socket_->get_socket_fd() < 0))
            {
                invalidate();
            }

            return;
        }

        /**
         * Destructor - automatically releases sockets.
         */
        ~LidarSocketPair()
        {
            invalidate();
        }

        // Delete copy operations
        LidarSocketPair(const LidarSocketPair &) = delete;
        LidarSocketPair &operator=(const LidarSocketPair &) = delete;

        // Allow move operations
        LidarSocketPair(LidarSocketPair &&other) noexcept
            : msop_socket_(std::move(other.msop_socket_)),
              difop_socket_(std::move(other.difop_socket_)) {}

        LidarSocketPair &operator=(LidarSocketPair &&other) noexcept
        {
            if (this != &other)
            {
                msop_socket_.reset();
                difop_socket_.reset();
                msop_socket_ = std::move(other.msop_socket_);
                difop_socket_ = std::move(other.difop_socket_);
            }
            return *this;
        }

        int get_msop_fd() const
        {
            return msop_socket_ ? msop_socket_->get_socket_fd() : -1;
        }

        int get_difop_fd() const
        {
            return difop_socket_ ? difop_socket_->get_socket_fd() : -1;
        }

        UDPListener *get_msop_socket() const { return msop_socket_.get(); }
        UDPListener *get_difop_socket() const { return difop_socket_.get(); }

        /**
         * Check if sockets are valid.
         */
        bool is_valid() const
        {
            return msop_socket_ && 
                   difop_socket_ &&
                   msop_socket_->is_open() &&
                   difop_socket_->is_open();
        }

        void invalidate()
        {
            msop_socket_.reset();
            difop_socket_.reset();
        }

    private:
        // Socket members only
        std::unique_ptr<UDPListener> msop_socket_;
        std::unique_ptr<UDPListener> difop_socket_;
    };

} // namespace mirena
