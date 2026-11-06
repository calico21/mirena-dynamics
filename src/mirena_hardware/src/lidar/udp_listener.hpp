#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <optional>

namespace mirena
{

class UDPListener
    {
    public:
        // If src addr and/or port is given, filters packets according to their source
        // The ports need to be given with htons
        UDPListener(in_addr_t self_addr,
                    uint16_t self_port,
                    bool auto_open = true,
                    std::optional<in_addr_t> src_addr = std::nullopt,
                    std::optional<uint16_t> src_port = std::nullopt)
        {
            ip_whitelist = src_addr;
            port_whitelist = src_port;
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = self_addr;
            server_addr.sin_port = self_port;

            if (auto_open)
            {
                open_sock();
            }
        }

        ~UDPListener()
        {
            close_sock();
        }

        void close_sock()
        {
            if (!_open)
                return;

            close(socket_fd);
            socket_fd = -1;
            _open = false;
        }

        // Tries to open socket
        // Check if this shit worked with .is_open()
        void open_sock()
        {
            if (_open)
            {
                return;
            }

            if (socket_fd < 0)
            {
                socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            }
            if (socket_fd < 0)return;

            int rcv_size = 26214400; // 25MB
            if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &rcv_size, sizeof(rcv_size)) < 0) //Set rcv buffer size
            {
                // Log warning but continue
            }

            int flags = fcntl(socket_fd, F_GETFL, 0);
            fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);

            if (bind(socket_fd, (const struct sockaddr *)&server_addr, sizeof(server_addr)))
            {
                close(socket_fd);
                socket_fd = -1;
                return;
            }
            _open = true;
        }

        bool is_open()
        {
            return _open;
        }

        int get_socket_fd()
        {
            return socket_fd;
        }

        // Returns the size of the packet read
        // Returns -1 if an error occured or if socket isnt open
        ssize_t _recvfrom(void *buffer, size_t buffer_len, int flags)
        {
            if (!_open)
            {
                return -1;
            }
            
            socklen_t cliaddr_len = sizeof(client_addr);
            ssize_t result = recvfrom(socket_fd, (char *)buffer, buffer_len,
                                      flags, (struct sockaddr *)&client_addr,
                                      &cliaddr_len);
            //std::cout << "readin" << std::endl;
            if (result < 0)
            {
                // No printing from here. This runs once per packet -- thousands
                // of times a second -- and the two std::cout traces that used to
                // live here ("errn: <n>" on every EAGAIN and "muerlto" on socket
                // death) went to stdout rather than the ROS log, so they were
                // invisible in a launch and unbounded in a terminal. The caller
                // learns what it needs from the return value and is_open().
                if (errno == EBADF ||      // Socket descriptor is invalid
                    errno == ENOTSOCK ||   // FD doesn't reference a socket
                    errno == EINVAL ||     // Invalid arguments (socket might be corrupted)
                    errno == ENOTCONN)     // Socket not connected (UDP normally doesn't get this, but handle anyway)
                {
                    close_sock();  // This sets _open = false and closes the FD
                }
                // For EAGAIN/EWOULDBLOCK (no data available), EINTR (interrupted),
                // or other transient errors, keep the socket open
                return -1;
            }

            bool matches_filters = true;
            if (ip_whitelist.has_value())
            {
                matches_filters = matches_filters && (ip_whitelist.value() == client_addr.sin_addr.s_addr);
            }
            if (port_whitelist.has_value())
            {
                matches_filters = matches_filters && (port_whitelist.value() == client_addr.sin_port);
            }
            if (matches_filters)
            {
                return result;
            }
            return 0;
        }

    private:
        bool _open = false;
        int socket_fd = -1;

        std::optional<in_addr_t> ip_whitelist;
        std::optional<in_port_t> port_whitelist;

        sockaddr_in server_addr;
        sockaddr_in client_addr;
    };
}