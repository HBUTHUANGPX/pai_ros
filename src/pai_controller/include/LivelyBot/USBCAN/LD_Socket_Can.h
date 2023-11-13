#ifndef _LD_SOCKET_CAN_H_
#define _LD_SOCKET_CAN_H_
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#include <net/if.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <termios.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>
typedef void (*syslog_t)(int priority, const char *format, ...);
class Tokenizer
{
public:
    Tokenizer(const std::string &source, const char *delimiters)
        : source_(source),
          delimiters_(delimiters),
          position_(source_.cbegin()) {}

    std::string next()
    {
        if (position_ == source_.end())
        {
            return std::string();
        }

        const auto start = position_;
        auto my_next = position_;
        bool found = false;
        for (; my_next != source_.end(); ++my_next)
        {
            if (std::strchr(delimiters_, *my_next) != nullptr)
            {
                position_ = my_next;
                ++position_;
                found = true;
                break;
            }
        }
        if (!found)
        {
            position_ = my_next;
        }
        return std::string(&*start, my_next - start);
    }

    std::string remaining() const
    {
        return std::string(&*position_, source_.end() - position_);
    }

private:
    const std::string source_;
    const char *const delimiters_;
    std::string::const_iterator position_;
};

class Socket_CAN
{
private:
    syslog_t syslogger = ::syslog;
    std::string tty;
    std::string ifname;
    bool verbose = false; // debug
    struct canfd_frame recv_frame = {};
    struct canfd_frame send_frame = {};
    std::ostringstream fdcanusb_buf;
    std::thread _thread;

public:
    Socket_CAN(std::string device_name, std::string CAN_port_name);
    ~Socket_CAN();
    fd_set rfds = {};
    int _socket, fd;
    struct sockaddr_can addr = {};
    void SetNonblock(int fd);
    void ErrorIf(bool value, const char *format, ...);
    void Append(std::ostream *ostr, const char *format, ...);
    bool StartsWith(const std::string &haystack, const std::string &needle);
    int ParseHexNybble(char c);
    int ParseHexByte(const char *value);
    int ParseCanData(const std::string &data, uint8_t *out);
};
Socket_CAN::Socket_CAN(std::string device_name, std::string CAN_port_name) : tty(device_name), ifname(CAN_port_name)
{
    syslogger(LOG_INFO, "starting on device %s", tty.c_str());
    printf("starting on device %s\n", tty.c_str());
    fd = ::open(tty.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    ErrorIf(fd < 0, "failed to open device %s", tty.c_str());
    // Set low-latency.
    {
        struct serial_struct serial = {};
        ErrorIf(::ioctl(fd, TIOCGSERIAL, &serial) < 0, "error getting serial flags");
        serial.flags |= ASYNC_LOW_LATENCY;
        ErrorIf(::ioctl(fd, TIOCSSERIAL, &serial) < 0, "error setting low latency");
    }
    std::cout << "set low latency ok" << std::endl;
    // Disable all kernel-side character processing.
    {
        struct termios tty;

        ErrorIf(::tcgetattr(fd, &tty) != 0, "error getting termios");

        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        ErrorIf(::tcsetattr(fd, TCSANOW, &tty) != 0, "error setting termios");
    }
    std::cout << "Disable all kernel-side character processing ok" << std::endl;
    SetNonblock(fd);
    // Now open the CAN interface.
    _socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ErrorIf(socket < 0, "error opening CAN socket");

    SetNonblock(_socket);

    struct ifreq ifr = {}; // 选择使用CAN口，可以在命令行中“ip link”查看可用的can口，比如can0，can1，vcan0，vcan1
    std::strncpy(&ifr.ifr_name[0], ifname.c_str(), sizeof(ifr.ifr_name) - 1);
    ErrorIf(::ioctl(_socket, SIOCGIFINDEX, &ifr) < 0, "could not find CAN '%s'", ifname.c_str());
    std::cout << "select can port ok : " + ifname << std::endl;

    int enable_canfd = 1;
    ErrorIf(::setsockopt(_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0, "could not set CAN-FD mode");
    std::cout << "ENALBE CANFD ok" << std::endl;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ErrorIf(::bind(_socket, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0, "could not bind to CAN if"); // 将 socket 绑定到一个特定的 CAN 接口
    std::cout << "SOCKET CANFD bind ok" << std::endl;
    std::cout << "addr.can_ifindex:" << addr.can_ifindex << std::endl;

    std::cout << "init ok" << std::endl;
    // _thread = std::thread(&Socket_CAN::thread, this);
    sleep(1);
}
int Socket_CAN::ParseHexNybble(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    return -1;
}
int Socket_CAN::ParseHexByte(const char *value)
{
    int high = ParseHexNybble(value[0]);
    if (high < 0)
    {
        return high;
    }
    int low = ParseHexNybble(value[1]);
    if (low < 0)
    {
        return low;
    }
    return (high << 4) | low;
}
int Socket_CAN::ParseCanData(const std::string &data, uint8_t *out)
{
    size_t to_read = std::min<size_t>(64 * 2, data.size());
    for (size_t i = 0; i < to_read; i += 2)
    {
        out[i / 2] = ParseHexByte(&data[i]);
    }
    return data.size() / 2;
}
bool Socket_CAN::StartsWith(const std::string &haystack, const std::string &needle)
{
    return haystack.substr(0, needle.size()) == needle;
}
void Socket_CAN::Append(std::ostream *ostr, const char *format, ...)
{
    char buf[1024] = {};
    va_list ap;

    va_start(ap, format);
    int count = ::vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    ostr->write(buf, count);
}
void Socket_CAN::SetNonblock(int fd)
{
    int flags = ::fcntl(fd, F_GETFL, 0);
    ErrorIf(flags < 0, "error getting flags");
    flags |= O_NONBLOCK;
    ErrorIf(::fcntl(fd, F_SETFL, flags), "error setting flags");
}
void Socket_CAN::ErrorIf(bool value, const char *format, ...)
{
    va_list ap;
    if (value)
    {
        va_start(ap, format);
        ::vfprintf(stderr, format, ap);
        va_end(ap);
        ::fprintf(stderr, "\n");
        ::perror("");
        std::exit(EXIT_FAILURE);
    }
    else
    {
        // Just consume this.
        char buf[1] = {};
        va_start(ap, format);
        ::vsnprintf(buf, sizeof(buf), format, ap);
        va_end(ap);
    }
}
Socket_CAN::~Socket_CAN()
{
}

#endif