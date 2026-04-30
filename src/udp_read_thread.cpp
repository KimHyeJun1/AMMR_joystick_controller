#include <joystick_controller/udp_read_thread.hpp>

UdpReadingThread::UdpReadingThread() {};

int UdpReadingThread::socketOpen(std::string ip, int port)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);

    int opt = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if(bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("bind failed");
        return -1;
    }

    std::cout << "UDP bind success : " << port << std::endl;

    return sock_;
}

void UdpReadingThread::socketClose() {
    close(sock_);
}

void UdpReadingThread::loop()
{
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    char buffer[1024];

    sockaddr_in client;
    socklen_t len = sizeof(client);

    while(rclcpp::ok())
    {
        int n = recvfrom(sock_, buffer, sizeof(buffer), 0,
                         (sockaddr*)&client, &len);

        if(buffer[0] == 0x01)
        {
            uint8_t reply[2] = {0x02, 0x01};                        //hello from client to server
            sendto(sock_, reply, 2, 0, (sockaddr*)&client, len);    // answer hello from server to client

        } else if (buffer[0] == 0x03) {
            // static uint8_t seq = 2;
            static uint8_t seq = buffer[1];
            uint8_t reply[2] = {0x02, seq += 1};
            sendto(sock_, reply, 2, 0, (sockaddr*)&client, len);

            //joystick
            joystick_x_ = buffer[2];
            joystick_y_ = buffer[3];
            joystick_btn_ = static_cast<uint8_t>(buffer[4]) | (static_cast<uint8_t>(buffer[5]) << 8);

            // std::cout << "joystick_btn_: " << joystick_btn_ << std::endl;
        }
    }
}

UdpReadingThread::~UdpReadingThread() {};